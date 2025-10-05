// SPDX-License-Identifier: Apache-2.0
// esphome_ghostuart - Generic UART MITM for ESPHome (A/B sides)

#include "mitm_component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <cstring>
#include <cmath>

namespace esphome {
namespace ghostuart {

static const char *TAG = "ghostuart";

// RX task: continuously services both UART sides so framing is independent of loop() latency
static void ghostuart_rx_task(void *param) {
  auto *self = reinterpret_cast<GhostUARTComponent*>(param);
  // Small cooperative delay to avoid watchdog issues
  const TickType_t tick_1ms = pdMS_TO_TICKS(1);
  for (;;) {
    self->rx_task_tick();
    vTaskDelay(tick_1ms);
  }
}

// ------------------------------- Utility ------------------------------------
// Calculate 8-bit LRC (two's complement of byte sum)
uint8_t GhostUARTComponent::calc_lrc_(const uint8_t *data, size_t len) {
  uint32_t s = 0;
  for (size_t i = 0; i < len; ++i) s += data[i];
  uint8_t sum8 = static_cast<uint8_t>(s & 0xFF);
  return static_cast<uint8_t>((0x100 - sum8) & 0xFF);
}

static inline uint32_t ceil_div_u32(uint32_t a, uint32_t b) {
  return (a + b - 1) / b;
}

// Auto / YAML timing computation
void GhostUARTComponent::recompute_timing_() {
  const uint32_t bits_per_char = 11;  // assume 8E1; safe for 8N1
  if (baud_ == 0) return;
  const float char_ms = (1000.0f * bits_per_char) / static_cast<float>(baud_);

  // Silence: use YAML if provided, otherwise 10 character times
  if (silence_ms_cfg_ > 0) {
    silence_ms_eff_ = silence_ms_cfg_;
  } else {
    uint32_t auto_ms = static_cast<uint32_t>(std::ceil(10.0f * char_ms));
    if (auto_ms < 3) auto_ms = 3;
    if (auto_ms > 300) auto_ms = 300;
    silence_ms_eff_ = auto_ms;
  }

  // Pre-listen: use YAML if provided, otherwise 0.75 character times (min 1 ms)
  if (pre_listen_ms_cfg_ > 0) {
    pre_listen_ms_eff_ = pre_listen_ms_cfg_;
  } else {
    uint32_t pre_ms = static_cast<uint32_t>(std::lround(0.75f * char_ms));
    if (pre_ms < 1) pre_ms = 1;
    pre_listen_ms_eff_ = pre_ms;
  }

  if (debug_enabled_) {
    ESP_LOGD(TAG, "recompute_timing baud=%u char_ms=%.3f -> silence_ms=%u pre_listen_ms=%u",
             baud_, char_ms, silence_ms_eff_, pre_listen_ms_eff_);
  }
}

// ------------------------------ Public API ---------------------------------
float GhostUARTComponent::get_var(const std::string &name) const {
  auto it = vars_.find(name);
  if (it == vars_.end() || !it->second.has_value) return NAN;
  return it->second.scaled;
}

bool GhostUARTComponent::send_template(const std::string &template_name,
                                       const std::unordered_map<std::string, std::string> &overrides) {
  auto it = std::find_if(templates_.begin(), templates_.end(),
                         [&](const FrameTemplate &t) { return t.name == template_name; });
  if (it == templates_.end()) {
    ESP_LOGW(TAG, "Template '%s' not found", template_name.c_str());
    return false;
  }
  InjectJob job;
  job.template_name = template_name;
  job.overrides = overrides;
  job.retries_left = DEFAULT_MAX_RETRIES;
  job.created_ms = millis();
  inject_queue_.push_back(job);
  if (debug_enabled_) {
    ESP_LOGD(TAG, "Enqueued inject template '%s' (overrides=%u)", template_name.c_str(), (unsigned)overrides.size());
  }
  return true;
}

void GhostUARTComponent::set_debug(bool en) {
  debug_enabled_ = en;
  ESP_LOGI(TAG, "Debug %s", en ? "enabled" : "disabled");
}

// ------------------------------ Setup / Loop -------------------------------
void GhostUARTComponent::setup() {
  // Initialize effective timings from config if explicitly set.
  if (silence_ms_cfg_ != 0) silence_ms_eff_ = silence_ms_cfg_;
  if (pre_listen_ms_cfg_ != 0) pre_listen_ms_eff_ = pre_listen_ms_cfg_;

  ESP_LOGI(TAG, "GhostUART setup – max_frame=%u, silence=%u ms, pre_listen=%u ms, debug=%s",
           max_frame_, silence_ms_eff_, pre_listen_ms_eff_, debug_enabled_ ? "true" : "false");

  // Initialize auto timings from baud if requested (silence/pre_listen == 0)
  recompute_timing_();

  // Spawn a dedicated RX task so we service UART buffers independent of loop() load
  //xTaskCreatePinnedToCore(ghostuart_rx_task, "ghostuart_rx", 4096, this, 10, nullptr, tskNO_AFFINITY);
}

void GhostUARTComponent::rx_task_tick() {
  // Service both sides (reads + small coalesce window)
  read_uart_(Direction::A_TO_B);
  read_uart_(Direction::B_TO_A);

  // Close frames on idle (do framing in RX task to decouple from loop timing)
  uint32_t now_ms = millis();
  for (int d = 0; d < 2; ++d) {
    Direction dir = static_cast<Direction>(d);
    auto &s = rx_[d];
    if (!s.buffer.empty()) {
      if ((now_ms - s.last_rx_ms) >= silence_ms_eff_) {
        if (!s.frame_ready) {
          s.frame_ready = true;
          on_silence_expired_(dir);
        }
      }
    }
  }
}

void GhostUARTComponent::loop() {
  // Drain and process frames completed by the RX task
  for (int d = 0; d < 2; ++d) {
    Direction dir = static_cast<Direction>(d);
    auto &q = ready_frames_[d];
    while (!q.empty()) {
      std::vector<uint8_t> frame = std::move(q.front());
      q.erase(q.begin());
      // Parse and forward now in loop context
      parse_and_store_(frame);
      forward_frame_(dir, frame);
    }
  }

  // Then handle injection
  process_inject_queue_();
}

// ------------------------------ UART read ----------------------------------
// Read available bytes from one side, update adaptive timing, and collect into a frame buffer.
void GhostUARTComponent::read_uart_(Direction dir) {
  uart::UARTComponent *rxu = rx_uart_(dir, uart_a_, uart_b_);
  if (!rxu) return;

  uint8_t buf[128];
  while (true) {
    int avail = rxu->available();
    if (avail <= 0) {
      // Coalesce: briefly wait for trailing bytes in a burst so we don't split frames
      // into 1-byte chunks when scheduling is slow.
      const uint32_t coalesce_us = 30000; // ~30 ms (tuned to capture slow inter-byte gaps)
      uint32_t t0 = micros();
      bool got_more = false;
      while ((micros() - t0) < coalesce_us) {
        int a2 = rxu->available();
        if (a2 > 0) { got_more = true; break; }
        delayMicroseconds(200);
      }
      if (!got_more) break; // truly nothing more to read now
      avail = rxu->available();
      if (avail <= 0) break;
    }

    int to_read = avail;
    if (to_read > static_cast<int>(sizeof(buf))) to_read = sizeof(buf);

    int r = rxu->read_array(buf, to_read);
    if (r <= 0) break;

    uint32_t now_ms = millis();
    auto &s = rx_[static_cast<int>(dir)];

    for (int i = 0; i < r; ++i) {
      if (s.buffer.size() < max_frame_) {
        s.buffer.push_back(buf[i]);
        s.interbyte_us.push_back(1);
      } else {
        // keep window sliding if we've overflowed the configured max_frame_
        s.buffer.erase(s.buffer.begin());
        s.buffer.push_back(buf[i]);
      }

      s.last_rx_ms = now_ms;
      s.frame_ready = false;
    }

    // Yield to let the UART driver and other tasks run
    delay(0);
  }
}

// ---------------------------- Silence expired ------------------------------
// Called when a frame is complete (no bytes received for silence_ms_eff_)
void GhostUARTComponent::on_silence_expired_(Direction dir) {
  auto &s = rx_[static_cast<int>(dir)];
  if (s.buffer.empty()) return;

  std::vector<uint8_t> frame = std::move(s.buffer);
  s.buffer.clear();
  s.interbyte_us.clear();
  s.frame_ready = false;
  frames_parsed_++;

  // Hand off the finished frame to loop() for parsing/forwarding
  enqueue_ready_frame_(dir, std::move(frame));
}

// ------------------------ Ready frame queue (RX task -> loop) ---------------
void GhostUARTComponent::enqueue_ready_frame_(Direction dir, std::vector<uint8_t> &&frame) {
  ready_frames_[static_cast<int>(dir)].push_back(std::move(frame));
}

// ---------------------------- Forwarding -----------------------------------
// Forward a complete frame from side A→B or B→A
void GhostUARTComponent::forward_frame_(Direction dir, const std::vector<uint8_t> &frame) {
  uart::UARTComponent *txu = tx_uart_(dir, uart_a_, uart_b_);
  if (!txu) {
    ESP_LOGW(TAG, "TX UART not configured (A/B)");
    return;
  }
  if (frame.empty()) return;

  txu->write_array(frame.data(), frame.size());
  frames_forwarded_[static_cast<int>(dir)]++;
  if (debug_enabled_) {
    // Build a short hex dump of up to the first 16 bytes
    char hex[3 * 16 + 1];
    int max_dump = frame.size() < 16 ? frame.size() : 16;
    int idx = 0;
    for (int i = 0; i < max_dump; ++i) {
      idx += snprintf(&hex[idx], sizeof(hex) - idx, "%02X%s", frame[i], (i + 1 < max_dump ? " " : ""));
      if (idx >= (int)sizeof(hex)) break;
    }
    hex[sizeof(hex) - 1] = '\0';

    if ((int)frame.size() <= max_dump) {
      ESP_LOGD(TAG, "Forwarded %u bytes (direction=%d) data=[%s]", (unsigned)frame.size(), static_cast<int>(dir), hex);
    } else {
      ESP_LOGD(TAG, "Forwarded %u bytes (direction=%d) data=[%s] ...", (unsigned)frame.size(), static_cast<int>(dir), hex);
    }
  }
}

// ----------------------------- Parsing -------------------------------------
// Parse received frame using mappings and update stored variables
void GhostUARTComponent::parse_and_store_(const std::vector<uint8_t> &frame) {
  for (const auto &m : mappings_) {
    if (!selector_match_(m.selector, frame)) continue;
    for (const auto &fd : m.fields) {
      std::string val_str;
      float val_scaled = NAN;
      std::vector<uint8_t> raw;
      bool ok = decode_field_(fd, frame, val_str, val_scaled, raw);
      if (!ok) {
        if (debug_enabled_) {
          ESP_LOGD(TAG, "Failed to decode field '%s' (mapping '%s')", fd.name.c_str(), m.name.c_str());
        }
        continue;
      }
      StoredVar &sv = vars_[fd.name];
      sv.has_value = true;
      sv.scaled = val_scaled;
      sv.last_raw = raw;
      sv.last_seen_ms = millis();
      sv.persistent = fd.persist;
      if (debug_enabled_) {
        ESP_LOGD(TAG, "Stored var %s = %s (raw=%uB)", fd.name.c_str(), val_str.c_str(), (unsigned)raw.size());
      }
    }
  }
}

bool GhostUARTComponent::selector_match_(const FrameSelector &sel, const std::vector<uint8_t> &frame) const {
  if (sel.prefix.empty()) return true;
  if (frame.size() < sel.prefix.size()) return false;
  for (size_t i = 0; i < sel.prefix.size(); ++i)
    if (frame[i] != sel.prefix[i]) return false;
  return true;
}

// ----------------------- Field decode / encode -----------------------------
bool GhostUARTComponent::decode_field_(const FieldDescriptor &fd,
                                       const std::vector<uint8_t> &frame,
                                       std::string &out_value_str, float &out_value_scaled,
                                       std::vector<uint8_t> &out_raw) const {
  if (fd.offset + fd.length > frame.size()) return false;
  const uint8_t *p = frame.data() + fd.offset;
  out_raw.assign(p, p + fd.length);

  switch (fd.format) {
    case FieldFormat::UINT8: {
      uint8_t v = out_raw[0];
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::INT8: {
      int8_t v = static_cast<int8_t>(out_raw[0]);
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::UINT16_LE: {
      uint16_t v = static_cast<uint16_t>(out_raw[0]) | (static_cast<uint16_t>(out_raw[1]) << 8);
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::UINT16_BE: {
      uint16_t v = (static_cast<uint16_t>(out_raw[0]) << 8) | static_cast<uint16_t>(out_raw[1]);
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::INT16_LE: {
      int16_t v = static_cast<int16_t>(out_raw[0] | (out_raw[1] << 8));
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::INT16_BE: {
      int16_t v = static_cast<int16_t>((out_raw[0] << 8) | out_raw[1]);
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::ASCII: {
      out_value_str.assign(reinterpret_cast<const char*>(out_raw.data()), out_raw.size());
      out_value_scaled = NAN;
      return true;
    }
    case FieldFormat::BCD: {
      long val = 0;
      for (auto b : out_raw) {
        int hi = (b >> 4) & 0x0F;
        int lo = b & 0x0F;
        val = val * 100 + hi * 10 + lo;
      }
      out_value_scaled = static_cast<float>(val) * fd.scale;
      out_value_str = std::to_string(val);
      return true;
    }
    case FieldFormat::BITFIELD: {
      uint32_t accum = 0;
      for (size_t i = 0; i < out_raw.size(); ++i)
        accum |= (static_cast<uint32_t>(out_raw[i]) << (8 * i));
      if (fd.mask) accum = (accum & fd.mask) >> fd.shift;
      out_value_scaled = static_cast<float>(accum) * fd.scale;
      out_value_str = std::to_string(accum);
      return true;
    }
    default:
      return false;
  }
}

bool GhostUARTComponent::encode_template_field_(const TemplateField &tf, const std::string &value_str,
                                                std::vector<uint8_t> &out_bytes) const {
  if (value_str.empty()) return false;
  float fv = std::stof(value_str);
  float rawf = fv / tf.scale;
  int32_t raw = static_cast<int32_t>(std::lround(rawf));
  out_bytes.clear();
  switch (tf.format) {
    case FieldFormat::INT16_LE: {
      int16_t r = static_cast<int16_t>(raw);
      out_bytes.push_back(static_cast<uint8_t>(r & 0xFF));
      out_bytes.push_back(static_cast<uint8_t>((r >> 8) & 0xFF));
      return true;
    }
    case FieldFormat::UINT16_LE: {
      uint16_t r = static_cast<uint16_t>(raw);
      out_bytes.push_back(static_cast<uint8_t>(r & 0xFF));
      out_bytes.push_back(static_cast<uint8_t>((r >> 8) & 0xFF));
      return true;
    }
    case FieldFormat::UINT8: {
      uint8_t r = static_cast<uint8_t>(raw & 0xFF);
      out_bytes.push_back(r);
      return true;
    }
    default:
      return false;
  }
}

// ------------------------ Build frame from template -------------------------
bool GhostUARTComponent::build_frame_from_template_(const FrameTemplate &tp,
                                                    const std::unordered_map<std::string, std::string> &overrides,
                                                    std::vector<uint8_t> &out_frame) {
  out_frame.clear();
  out_frame.insert(out_frame.end(), tp.fixed_prefix.begin(), tp.fixed_prefix.end());

  for (const auto &ph : tp.placeholders) {
    std::string val_str;
    auto it = overrides.find(ph.name);
    if (it != overrides.end()) {
      val_str = it->second;
    } else {
      auto itv = vars_.find(ph.name);
      if (itv == vars_.end() || !itv->second.has_value) {
        ESP_LOGW(TAG, "Template field '%s' missing and not found in stored vars", ph.name.c_str());
        return false;
      }
      val_str = std::to_string(itv->second.scaled);
    }
    std::vector<uint8_t> enc;
    if (!encode_template_field_(ph, val_str, enc)) {
      ESP_LOGW(TAG, "Failed to encode template field '%s'", ph.name.c_str());
      return false;
    }
    out_frame.insert(out_frame.end(), enc.begin(), enc.end());
  }

  if (tp.append_lrc) {
    uint8_t lrc = calc_lrc_(out_frame.data(), out_frame.size());
    out_frame.push_back(lrc);
  }
  return true;
}

// ------------------------ Inject queue processing --------------------------
// Wait for both sides to be idle, then send next frame from the injection queue
bool GhostUARTComponent::bus_idle_() const {
  uint32_t now = millis();
  for (int d = 0; d < 2; ++d) {
    const auto &s = rx_[d];
    if (!s.buffer.empty()) return false;
    if ((now - s.last_rx_ms) < silence_ms_eff_) return false;
  }
  return true;
}

void GhostUARTComponent::process_inject_queue_() {
  if (inject_queue_.empty()) return;
  if (!inject_enabled_) return;

  InjectJob job = inject_queue_.front();

  auto it_tpl = std::find_if(templates_.begin(), templates_.end(),
                             [&](const FrameTemplate &t) { return t.name == job.template_name; });
  if (it_tpl == templates_.end()) {
    ESP_LOGW(TAG, "Inject job: template '%s' not found, dropping", job.template_name.c_str());
    inject_queue_.erase(inject_queue_.begin());
    return;
  }

  // Wait until both directions are idle
  if (!bus_idle_()) return;

  // Short pre-listen
  delay(pre_listen_ms_eff_);
  if (!bus_idle_()) return;

  std::vector<uint8_t> frame;
  if (!build_frame_from_template_(*it_tpl, job.overrides, frame)) {
    ESP_LOGW(TAG, "Failed to build frame from template '%s'", job.template_name.c_str());
    inject_queue_.erase(inject_queue_.begin());
    return;
  }

  // Send on side B by default (typical "remote emulation" towards bus)
  uart::UARTComponent *txb = uart_b_;
  if (!txb) {
    ESP_LOGW(TAG, "No side B UART configured for inject");
    inject_queue_.erase(inject_queue_.begin());
    return;
  }

  txb->write_array(frame.data(), frame.size());
  ESP_LOGI(TAG, "Injected frame (%u bytes) via template '%s'", (unsigned)frame.size(), job.template_name.c_str());

  inject_queue_.erase(inject_queue_.begin());
}

// --------------------------------- EOF ------------------------------------
}  // namespace ghostuart
}  // namespace esphome
