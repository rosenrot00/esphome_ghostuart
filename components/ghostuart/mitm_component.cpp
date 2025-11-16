// SPDX-License-Identifier: Apache-2.0
// esphome_ghostuart - Generic UART MITM for ESPHome (A/B sides)

#include "mitm_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <cstring>
#include <cmath>

namespace esphome {
namespace ghostuart {


static const char *TAG = "ghostuart";

// ------------------------------- RX task -----------------------------------
// Dedicated RX task — calls rx_task_tick() on the component instance.
// It sleeps a little between ticks to be cooperative with other tasks.
static void ghostuart_rx_task(void *param) {
  auto *self = reinterpret_cast<GhostUARTComponent *>(param);
  const TickType_t tick_1ms = pdMS_TO_TICKS(1);
  for (;;) {
    if (!self->rx_task_is_enabled()) {
      // If disabled (for OTA or similar), sleep longer to reduce load.
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    self->rx_task_tick();
    vTaskDelay(tick_1ms);
  }
}

// ------------------------------- Utility ------------------------------------
// Calculate 8-bit LRC (two's complement of byte sum)
uint8_t GhostUARTComponent::calc_lrc_(const uint8_t *data, size_t len) {
  uint32_t s = 0;
  for (size_t i = 0; i < len; ++i)
    s += data[i];
  uint8_t sum8 = static_cast<uint8_t>(s & 0xFF);
  return static_cast<uint8_t>((0x100 - sum8) & 0xFF);
}


// Auto / YAML timing computation
void GhostUARTComponent::recompute_timing_() {
  const uint32_t bits_per_char = 11;  // conservative: 8 data + parity + start/stop
  if (baud_ == 0) return;
  const float char_ms = (1000.0f * bits_per_char) / static_cast<float>(baud_);

  // Silence: YAML if provided, otherwise N character times (10 here by default)
  if (silence_ms_cfg_ > 0) {
    silence_ms_eff_ = silence_ms_cfg_;
  } else {
    uint32_t auto_ms = static_cast<uint32_t>(std::ceil(10.0f * char_ms));
    if (auto_ms < 3) auto_ms = 3;
    if (auto_ms > 300) auto_ms = 300;
    silence_ms_eff_ = auto_ms;
  }

  // Pre-listen: YAML if provided, otherwise ~0.75 char times
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
  // Apply explicit YAML values if provided; recompute otherwise.
  if (silence_ms_cfg_ != 0) silence_ms_eff_ = silence_ms_cfg_;
  if (pre_listen_ms_cfg_ != 0) pre_listen_ms_eff_ = pre_listen_ms_cfg_;

  ESP_LOGI(TAG, "GhostUART setup – max_frame=%u, silence=%u ms, pre_listen=%u ms, debug=%s",
           max_frame_, silence_ms_eff_, pre_listen_ms_eff_, debug_enabled_ ? "true" : "false");

  // Compute auto timing if needed
  recompute_timing_();

  // If IDF/native UART mode requested, initialize UART drivers with event queues
  if (use_idf_driver_) {
    bool ok_a = idf_init_uart_(idf_uart_num_a_, idf_tx_pin_a_, idf_rx_pin_a_,
                               idf_rx_buf_a_, idf_rx_timeout_chars_a_, idf_uart_queue_a_, "A");
    bool ok_b = idf_init_uart_(idf_uart_num_b_, idf_tx_pin_b_, idf_rx_pin_b_,
                               idf_rx_buf_b_, idf_rx_timeout_chars_b_, idf_uart_queue_b_, "B");
    idf_driver_installed_a_ = ok_a;
    idf_driver_installed_b_ = ok_b;
  }

  // Spawn RX task to service UARTs independently of the main loop
  xTaskCreatePinnedToCore(ghostuart_rx_task, "ghostuart_rx", 4096, this, 4, &rx_task_handle_, tskNO_AFFINITY);
}

void GhostUARTComponent::rx_task_tick() {
  // Only IDF driver mode is supported; service hardware event queues
  if (idf_driver_installed_a_ && idf_uart_queue_a_) idf_service_events_(idf_uart_num_a_, idf_uart_queue_a_, Direction::A_TO_B);
  if (idf_driver_installed_b_ && idf_uart_queue_b_) idf_service_events_(idf_uart_num_b_, idf_uart_queue_b_, Direction::B_TO_A);

  // After draining events, check idle closure (as a safety net)
  uint32_t now_ms = millis();
  for (int d = 0; d < 2; ++d) {
    Direction dir = static_cast<Direction>(d);
    auto &s = rx_[d];
    if (!s.buffer.empty() && (now_ms - s.last_rx_ms) >= silence_ms_eff_ && !s.frame_ready) {
      s.frame_ready = true;
      on_silence_expired_(dir);
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

      // Apply per-instance filters
      FilterAction act = match_filters_(dir, frame);
      if (act == FilterAction::DROP) {
        // Neither log nor forward
        continue;
      }

      bool do_log = (act != FilterAction::FORWARD_ONLY);
      bool do_forward = (act != FilterAction::LOG_ONLY);

      // RX logging (frame-level)
      if (debug_enabled_ && do_log) {
        // Build a short hex dump of up to 32 bytes
        char hex_rx[3 * 32 + 1];
        int max_dump_rx = frame.size() < 32 ? frame.size() : 32;
        int idx_rx = 0;
        for (int i = 0; i < max_dump_rx; ++i) {
          idx_rx += snprintf(&hex_rx[idx_rx], sizeof(hex_rx) - idx_rx,
                             "%02X%s", frame[i], (i + 1 < max_dump_rx ? " " : ""));
          if (idx_rx >= (int)sizeof(hex_rx)) break;
        }
        hex_rx[sizeof(hex_rx) - 1] = '\0';

        // dir==A_TO_B bedeutet: empfangen auf A, weiter nach B -> Label RX(A)
        ESP_LOGD(TAG, "RX(%c) %u bytes data=[%s]%s",
                 (dir == Direction::A_TO_B ? 'A' : 'B'),
                 (unsigned)frame.size(),
                 hex_rx,
                 (frame.size() > (size_t)max_dump_rx ? " ..." : ""));
      }

      // Always parse (filters do not affect variable extraction)
      parse_and_store_(frame);

      // Forward only if permitted by filter
      if (do_forward) {
        forward_frame_(dir, frame);
      }
    }
  }

  // Then handle injection queue (sends after frames processed)
  process_inject_queue_();
}



// ---------------------------- Silence expired ------------------------------
// Called by RX task when a frame is closed (no new bytes for silence_ms_eff_)
void GhostUARTComponent::on_silence_expired_(Direction dir) {
  auto &s = rx_[static_cast<int>(dir)];
  if (s.buffer.empty()) return;

  std::vector<uint8_t> frame = std::move(s.buffer);
  s.buffer.clear();
  s.frame_ready = false;
  frames_parsed_++;

  // Enqueue for loop() to process (parse + forward)
  enqueue_ready_frame_(dir, std::move(frame));
}

// ------------------------ Ready frame queue (RX task -> loop) ---------------
void GhostUARTComponent::enqueue_ready_frame_(Direction dir, std::vector<uint8_t> &&frame) {
  ready_frames_[static_cast<int>(dir)].push_back(std::move(frame));
}

// ------------------------ Filter matching -----------------------------------
FilterAction GhostUARTComponent::match_filters_(Direction dir, const std::vector<uint8_t> &frame) const {
  for (const auto &r : filters_) {
    if (r.direction != dir) continue;
    if (frame.size() < r.prefix.size()) continue;
    bool ok = true;
    for (size_t i = 0; i < r.prefix.size(); ++i) {
      if (frame[i] != r.prefix[i]) { ok = false; break; }
    }
    if (ok) return r.action;
  }
  return FilterAction::NORMAL;
}

// ------------------------ Filter rule addition ------------------------------
void GhostUARTComponent::add_filter_rule(uint8_t dir_code,
                                         const std::vector<uint8_t> &prefix,
                                         uint8_t action_code) {
  Direction dir = (dir_code == 1) ? Direction::B_TO_A : Direction::A_TO_B;
  FilterAction act;
  switch (action_code) {
    case 1: act = FilterAction::DROP;         break;
    case 2: act = FilterAction::FORWARD_ONLY; break;
    case 3: act = FilterAction::LOG_ONLY;     break;
    case 0:
    default: act = FilterAction::NORMAL;      break;
  }

  FilterRule rule;
  rule.direction = dir;
  rule.prefix = prefix;
  rule.action = act;
  filters_.push_back(std::move(rule));

  if (debug_enabled_) {
    ESP_LOGI(TAG, "Added filter: dir=%d action=%d prefix_len=%u", (int)dir, (int)act, (unsigned)prefix.size());
  }
}

// ---------------------------- Forwarding -----------------------------------
void GhostUARTComponent::forward_frame_(Direction dir, const std::vector<uint8_t> &frame) {
  if (frame.empty()) return;

  // Debug before sending (always dump up to 32 bytes)
  if (debug_enabled_) {
    char hex[3 * 32 + 1];
    int max_dump = frame.size() < 32 ? frame.size() : 32;
    int idx = 0;
    for (int i = 0; i < max_dump; ++i) {
      idx += snprintf(&hex[idx], sizeof(hex) - idx, "%02X%s", frame[i], (i + 1 < max_dump ? " " : ""));
      if (idx >= (int)sizeof(hex)) break;
    }
    hex[sizeof(hex) - 1] = '\0';
    ESP_LOGD(TAG, "TX(%c) %u bytes data=[%s]%s",
             (dir == Direction::A_TO_B ? 'B' : 'A'), (unsigned)frame.size(), hex,
             (frame.size() > (size_t)max_dump ? " ..." : ""));
  }

  int port = (dir == Direction::A_TO_B) ? idf_uart_num_b_ : idf_uart_num_a_;
  bool installed = (dir == Direction::A_TO_B) ? idf_driver_installed_b_ : idf_driver_installed_a_;
  if (port < 0 || !installed) {
    ESP_LOGW(TAG, "TX IDF port not ready (port=%d, installed=%d)", port, (int)installed);
    return;
  }
  int written = uart_write_bytes((uart_port_t)port, frame.data(), frame.size());
  if (written < 0) {
    ESP_LOGW(TAG, "uart_write_bytes failed on UART%d (len=%u)", port, (unsigned)frame.size());
    return;
  }
  frames_forwarded_[static_cast<int>(dir)]++;
}

// ----------------------------- Parsing -------------------------------------
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
      if (out_raw.size() < 2) return false;
      uint16_t v = static_cast<uint16_t>(out_raw[0]) | (static_cast<uint16_t>(out_raw[1]) << 8);
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::UINT16_BE: {
      if (out_raw.size() < 2) return false;
      uint16_t v = (static_cast<uint16_t>(out_raw[0]) << 8) | static_cast<uint16_t>(out_raw[1]);
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::INT16_LE: {
      if (out_raw.size() < 2) return false;
      int16_t v = static_cast<int16_t>(static_cast<uint16_t>(out_raw[0]) | (static_cast<uint16_t>(out_raw[1]) << 8));
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::INT16_BE: {
      if (out_raw.size() < 2) return false;
      int16_t v = static_cast<int16_t>((static_cast<uint16_t>(out_raw[0]) << 8) | static_cast<uint16_t>(out_raw[1]));
      out_value_scaled = static_cast<float>(v) * fd.scale;
      out_value_str = std::to_string(v);
      return true;
    }
    case FieldFormat::ASCII: {
      out_value_str.assign(reinterpret_cast<const char *>(out_raw.data()), out_raw.size());
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
bool GhostUARTComponent::bus_idle_(Direction dir) const {
  uint32_t now = millis();
  const auto &s = rx_[static_cast<int>(dir)];
  if (!s.buffer.empty()) return false;
  if ((now - s.last_rx_ms) < silence_ms_eff_) return false;
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

  // Determine target side: default B, allow override via {"_target":"A"|"B"}
  Direction target_dir = Direction::A_TO_B; // A->B means we will TX on side B
  auto it_tgt = job.overrides.find("_target");
  if (it_tgt != job.overrides.end()) {
    const std::string &t = it_tgt->second;
    if (!t.empty() && (t[0] == 'A' || t[0] == 'a')) {
      target_dir = Direction::B_TO_A; // TX on side A
    } else if (!t.empty() && (t[0] == 'B' || t[0] == 'b')) {
      target_dir = Direction::A_TO_B; // TX on side B
    }
  }

  // Wait until bus is idle on the target side
  if (!bus_idle_(target_dir)) return;

  // Pre-listen time
  delay(pre_listen_ms_eff_);
  if (!bus_idle_(target_dir)) return;

  std::vector<uint8_t> frame;
  if (!build_frame_from_template_(*it_tpl, job.overrides, frame)) {
    ESP_LOGW(TAG, "Failed to build frame from template '%s'", job.template_name.c_str());
    inject_queue_.erase(inject_queue_.begin());
    return;
  }

  // Reuse normal TX path (handles IDF/legacy and logging):
  forward_frame_(target_dir, frame);

  ESP_LOGI(TAG, "Injected frame (%u bytes) via template '%s' to %c",
           (unsigned)frame.size(), job.template_name.c_str(),
           (target_dir == Direction::A_TO_B ? 'B' : 'A'));

  inject_queue_.erase(inject_queue_.begin());
}

// ------------------------ RX task control ---------------------------------
void GhostUARTComponent::set_rx_task_enabled(bool en) {
  rx_task_enabled_ = en;
  if (rx_task_handle_ == nullptr) return;
  if (!en) {
    vTaskSuspend(rx_task_handle_);
    ESP_LOGI(TAG, "RX task suspended");
  } else {
    vTaskResume(rx_task_handle_);
    ESP_LOGI(TAG, "RX task resumed");
  }
}

// ------------------------ IDF UART helpers ---------------------------------
// Initialize a native IDF UART with driver and event queue. Pins optional.
bool GhostUARTComponent::idf_init_uart_(int uart_num, int tx_pin, int rx_pin,
                                        uint32_t rx_buf, uint8_t timeout_chars,
                                        QueueHandle_t &out_queue, const char *side_tag) {
  if (uart_num < 0) {
    ESP_LOGW(TAG, "%s: IDF uart_num not set; skipping", side_tag);
    return false;
  }

  uart_config_t cfg{};
  cfg.baud_rate = static_cast<int>(baud_);
  cfg.data_bits = UART_DATA_8_BITS;
  // Set parity based on parity_mode_
  switch (parity_mode_) {
    case 0: cfg.parity = UART_PARITY_DISABLE; break; // NONE
    case 2: cfg.parity = UART_PARITY_ODD;     break; // ODD
    case 1: default: cfg.parity = UART_PARITY_EVEN;  break; // EVEN
  }
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  // Some ESP-IDF versions don’t expose UART_SCLK_APB; leave default if unavailable
  #ifdef UART_SCLK_APB
    cfg.source_clk = UART_SCLK_APB;
  #elif defined(UART_SCLK_DEFAULT)
    cfg.source_clk = UART_SCLK_DEFAULT;
  #endif

  esp_err_t err = uart_param_config(static_cast<uart_port_t>(uart_num), &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s: uart_param_config err=%d", side_tag, (int)err);
    return false;
  }

  // If pins provided, apply them. Otherwise assume pinmux already set externally.
  if (tx_pin >= 0 && rx_pin >= 0) {
    err = uart_set_pin(static_cast<uart_port_t>(uart_num), tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s: uart_set_pin err=%d", side_tag, (int)err);
      return false;
    }
  } else {
    ESP_LOGW(TAG, "%s: uart pins not set via IDF; leaving existing pinmux", side_tag);
  }

  // Install driver with RX buffer and an event queue (small queue depth)
  // tx buffer set to 0 (we use uart_write_bytes via driver when needed)
  // Install driver with RX and a small TX buffer; create an event queue
    err = uart_driver_install(static_cast<uart_port_t>(uart_num), static_cast<int>(rx_buf), 256, 20, &out_queue, 0);
    if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s: uart_driver_install err=%d", side_tag, (int)err);
    return false;
    }

    // Ensure UART mode is normal UART (not RS485 etc.)
    err = uart_set_mode(static_cast<uart_port_t>(uart_num), UART_MODE_UART);
    if (err != ESP_OK) {
    ESP_LOGW(TAG, "%s: uart_set_mode err=%d", side_tag, (int)err);
    }

    // Configure RX timeout in chars (idle detection)
    if (timeout_chars > 0) {
    err = uart_set_rx_timeout(static_cast<uart_port_t>(uart_num), timeout_chars);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: uart_set_rx_timeout err=%d", side_tag, (int)err);
        // not fatal — continue
    }
    }

  ESP_LOGI(TAG, "%s: IDF UART%d init ok (baud=%u, timeout_chars=%u, rx_buf=%u)", side_tag, uart_num, baud_, timeout_chars, (unsigned)rx_buf);
  return true;
}

// Service events from an IDF UART event queue.
// Reads UART_DATA into rx_ buffer and closes frame on RXFIFO_TOUT / BREAK events.
void GhostUARTComponent::idf_service_events_(int uart_num, QueueHandle_t queue, Direction dir) {
  if (!queue) return;

  uart_event_t evt;
  // Drain quickly; do not block here (outer RX task is periodic)
  while (xQueueReceive(queue, &evt, 0) == pdTRUE) {
    switch (evt.type) {
      case UART_DATA: {
        size_t rxlen = 0;
        uart_get_buffered_data_len(static_cast<uart_port_t>(uart_num), &rxlen);
        if (rxlen == 0) rxlen = evt.size;
        if (rxlen > 0) {
          std::vector<uint8_t> tmp(rxlen);
          int r = uart_read_bytes(static_cast<uart_port_t>(uart_num), tmp.data(), rxlen, 0);
          if (r > 0) {
            auto &s = rx_[static_cast<int>(dir)];
            uint32_t now_ms = millis();
            for (int i = 0; i < r; ++i) {
              if (s.buffer.size() < max_frame_) s.buffer.push_back(tmp[i]);
              else { s.buffer.erase(s.buffer.begin()); s.buffer.push_back(tmp[i]); }
              s.last_rx_ms = now_ms;
              s.frame_ready = false;
            }
          }
        }
        break;
      }
      // Removed UART_RXFIFO_TOUT event; rely on silence close and BREAK event.
      case UART_BREAK: {
        // Hardware idle/break → close frame immediately
        auto &s = rx_[static_cast<int>(dir)];
        if (!s.buffer.empty() && !s.frame_ready) {
          s.frame_ready = true;
          on_silence_expired_(dir);
        }
        break;
      }
      case UART_FIFO_OVF:
      case UART_BUFFER_FULL: {
        ESP_LOGW(TAG, "UART%d overflow / buffer full (dir=%d)", uart_num, static_cast<int>(dir));
        uart_flush(static_cast<uart_port_t>(uart_num));
        break;
      }
      default:
        break;
    }
  }
}

// --------------------------------- EOF ------------------------------------
}  // namespace ghostuart
}  // namespace esphome
