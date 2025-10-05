// SPDX-License-Identifier: Apache-2.0
// esphome_ghostuart - Generic UART MITM for ESPHome (A/B sides)

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <cmath>

namespace esphome {
namespace ghostuart {

// Defaults (can be overridden via YAML setters)
constexpr uint32_t DEFAULT_BAUD = 9600;
constexpr uint16_t DEFAULT_MAX_FRAME = 512;
constexpr uint16_t DEFAULT_RX_BUF = 4096;
constexpr uint16_t DEFAULT_TX_BUF = 2048;
constexpr uint32_t DEFAULT_SILENCE_MS = 15;   // 0 = auto
constexpr uint32_t DEFAULT_PRE_LISTEN_MS = 3; // 0 = auto
constexpr uint8_t  DEFAULT_MAX_RETRIES = 2;

enum class Direction : uint8_t { A_TO_B = 0, B_TO_A = 1 };

enum class FieldFormat : uint8_t {
  UINT8, INT8, UINT16_LE, UINT16_BE, INT16_LE, INT16_BE,
  UINT32_LE, UINT32_BE, ASCII, BCD, BITFIELD
};

struct FieldDescriptor {
  std::string name;
  uint16_t    offset{0};
  uint8_t     length{1};
  FieldFormat format{FieldFormat::UINT8};
  float       scale{1.0f};
  uint32_t    mask{0};
  uint8_t     shift{0};
  bool        persist{false};
  std::string transform;
};

struct FrameSelector {
  std::vector<uint8_t> prefix;
};

struct Mapping {
  std::string                  name;
  FrameSelector                selector;
  std::vector<FieldDescriptor> fields;
};

struct TemplateField {
  std::string name;
  FieldFormat format{FieldFormat::INT16_LE};
  float       scale{1.0f};
};

struct FrameTemplate {
  std::string name;
  std::vector<uint8_t> fixed_prefix;
  std::vector<TemplateField> placeholders;
  bool append_lrc{true};
  bool expect_aux_after{false};
  FrameSelector aux_after_selector;
};

struct InjectJob {
  std::string template_name;
  std::unordered_map<std::string, std::string> overrides;
  uint8_t  retries_left{DEFAULT_MAX_RETRIES};
  uint32_t created_ms{0};
};

struct StoredVar {
  bool        has_value{false};
  float       scaled{NAN};
  std::vector<uint8_t> last_raw;
  uint32_t    last_seen_ms{0};
  bool        persistent{false};
};

class GhostUARTComponent : public Component {
 public:
  // Wiring
  void set_uart_a(uart::UARTComponent *u) { uart_a_ = u; }
  void set_uart_b(uart::UARTComponent *u) { uart_b_ = u; }

  // Shared baud rate for A and B (used for auto timing; UARTs themselves are set in YAML)
  void set_baud(uint32_t baud) { baud_ = baud; recompute_timing_(); }

  // Timing/config (YAML setters – 0 means auto)
  void set_silence_ms(uint32_t ms) { silence_ms_cfg_ = ms; recompute_timing_(); }
  void set_pre_listen_ms(uint32_t ms) { pre_listen_ms_cfg_ = ms; recompute_timing_(); }
  void set_max_frame(uint16_t n) { max_frame_ = n; }

  // Mappings & templates
  void add_mapping(const Mapping &m) { mappings_.push_back(m); }
  void add_template(const FrameTemplate &t) { templates_.push_back(t); }

  // Variable access (scaled). Returns NAN if unknown.
  float get_var(const std::string &name) const;

  // Enqueue an inject job by template name and optional overrides.
  bool send_template(const std::string &template_name,
                     const std::unordered_map<std::string, std::string> &overrides);

  // Stats
  uint32_t get_frames_forwarded(Direction dir) const { return frames_forwarded_[static_cast<int>(dir)]; }
  uint32_t get_frames_parsed() const { return frames_parsed_; }
  uint32_t get_checksum_errors() const { return checksum_errors_; }

  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  // UARTs
  uart::UARTComponent *uart_a_{nullptr}; // side A
  uart::UARTComponent *uart_b_{nullptr}; // side B

  // Runtime config
  uint32_t baud_{DEFAULT_BAUD};
  uint16_t max_frame_{DEFAULT_MAX_FRAME};
  uint32_t silence_ms_cfg_{DEFAULT_SILENCE_MS};       // 0 = auto
  uint32_t pre_listen_ms_cfg_{DEFAULT_PRE_LISTEN_MS}; // 0 = auto
  uint32_t silence_ms_eff_{DEFAULT_SILENCE_MS};       // effective value after auto calculation
  uint32_t pre_listen_ms_eff_{DEFAULT_PRE_LISTEN_MS};

  // RX state
  struct RxState {
    std::vector<uint8_t>  buffer;
    std::vector<uint16_t> interbyte_us;
    uint32_t last_rx_ms{0};
    bool     frame_ready{false};
  } rx_[2];

  // Injection
  std::vector<InjectJob> inject_queue_;
  bool inject_enabled_{false};

  // Mappings/templates
  std::vector<Mapping> mappings_;
  std::vector<FrameTemplate> templates_;

  // Vars & stats
  std::unordered_map<std::string, StoredVar> vars_;
  uint32_t frames_forwarded_[2]{0,0};
  uint32_t frames_parsed_{0};
  uint32_t checksum_errors_{0};

  // Internals
  void read_uart_(Direction dir);
  void on_silence_expired_(Direction dir);
  void forward_frame_(Direction dir, const std::vector<uint8_t> &frame);
  void parse_and_store_(const std::vector<uint8_t> &frame);
  bool selector_match_(const FrameSelector &sel, const std::vector<uint8_t> &frame) const;

  bool decode_field_(const FieldDescriptor &fd,
                     const std::vector<uint8_t> &frame,
                     std::string &out_value_str, float &out_value_scaled,
                     std::vector<uint8_t> &out_raw) const;

  bool encode_template_field_(const TemplateField &tf, const std::string &value_str,
                              std::vector<uint8_t> &out_bytes) const;

  static uint8_t calc_lrc_(const uint8_t *data, size_t len);
  bool build_frame_from_template_(const FrameTemplate &tp,
                                  const std::unordered_map<std::string, std::string> &overrides,
                                  std::vector<uint8_t> &out_frame);
  void process_inject_queue_();

  bool bus_idle_() const;
  void recompute_timing_(); // Auto: derive from baud_

  static uart::UARTComponent *rx_uart_(Direction dir, uart::UARTComponent *a, uart::UARTComponent *b) {
    return (dir == Direction::A_TO_B) ? a : b;
  }
  static uart::UARTComponent *tx_uart_(Direction dir, uart::UARTComponent *a, uart::UARTComponent *b) {
    return (dir == Direction::A_TO_B) ? b : a;
  }
};

}  // namespace ghostuart
}  // namespace esphome
