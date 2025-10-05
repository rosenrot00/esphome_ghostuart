// SPDX-License-Identifier: Apache-2.0
// esphome_ghostuart - Generic UART MITM for ESPHome
// Header: MITM core types, public API, and configuration surfaces.

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>
#include <cstdint>
#include <functional>
#include <unordered_map>

namespace esphome {
namespace ghostuart {

// ---- Compile-time defaults (can be overridden via YAML) --------------------
constexpr uint32_t DEFAULT_BAUD = 9600;
constexpr uint16_t DEFAULT_MAX_FRAME = 512;
constexpr uint16_t DEFAULT_RX_BUF = 4096;
constexpr uint16_t DEFAULT_TX_BUF = 2048;
constexpr uint32_t DEFAULT_SILENCE_MS = 15;   // Inter-byte gap to close a frame
constexpr uint32_t DEFAULT_PRE_LISTEN_MS = 3; // Short check before sending
constexpr uint8_t  DEFAULT_MAX_RETRIES = 2;

// ---- Utility enums ---------------------------------------------------------
enum class Direction : uint8_t { A_TO_B = 0, B_TO_A = 1 };

// ---- Field mapping: extract values from frames -----------------------------
enum class FieldFormat : uint8_t {
  UINT8, INT8, UINT16_LE, UINT16_BE, INT16_LE, INT16_BE,
  UINT32_LE, UINT32_BE, ASCII, BCD, BITFIELD
};

struct FieldDescriptor {
  std::string name;      // variable name (e.g., "remote_setpoint")
  uint16_t    offset{0}; // byte offset within frame (0-based)
  uint8_t     length{1}; // bytes (1..4 typical)
  FieldFormat format{FieldFormat::UINT8};
  float       scale{1.0f};   // multiply factor after decode
  uint32_t    mask{0};       // optional bit-mask (for BITFIELD)
  uint8_t     shift{0};      // optional shift for masked bits
  bool        persist{false}; // whether to persist across reboots
  std::string transform;     // optional transform name (implemented in component)
};

struct FrameSelector {
  // Minimal v1: prefix match (hex bytes). Further selectors can be added later.
  std::vector<uint8_t> prefix; // if non-empty, frame must start with this prefix
};

struct Mapping {
  std::string          name;     // logical group (e.g., "remote_commands")
  FrameSelector        selector; // which frames to parse
  std::vector<FieldDescriptor> fields;
};

// ---- Template-based injection ---------------------------------------------
struct TemplateField {
  // A named placeholder to be filled from stored variables or overrides.
  std::string name;
  FieldFormat format{FieldFormat::INT16_LE};
  float       scale{1.0f};      // physical -> raw conversion uses 1/scale
};

struct FrameTemplate {
  std::string name; // e.g., "setpoint_cmd"
  std::vector<uint8_t> fixed_prefix;
  std::vector<TemplateField> placeholders;
  bool append_lrc{true};
  bool expect_aux_after{false};
  FrameSelector aux_after_selector; // optional pattern to increase success rate
};

// ---- Injection job ---------------------------------------------------------
struct InjectJob {
  std::string template_name;
  std::unordered_map<std::string, std::string> overrides;
  uint8_t  retries_left{DEFAULT_MAX_RETRIES};
  uint32_t created_ms{0};
};

// ---- Stored variable -------------------------------------------------------
struct StoredVar {
  bool        has_value{false};
  float       scaled{NAN};              // e.g., °C
  std::vector<uint8_t> last_raw;       // raw bytes as captured
  uint32_t    last_seen_ms{0};
  bool        persistent{false};
};

// ---- GhostUART Component ---------------------------------------------------
class GhostUARTComponent : public Component {
 public:
  // Wiring: UART A = side A, UART B = side B
  void set_uart_a(uart::UARTComponent *u) { uart_a_ = u; }
  void set_uart_b(uart::UARTComponent *u) { uart_b_ = u; }

  // Timing/config
  void set_silence_ms(uint32_t ms) { silence_ms_ = ms; }
  void set_pre_listen_ms(uint32_t ms) { pre_listen_ms_ = ms; }
  void set_max_frame(uint16_t n) { max_frame_ = n; }

  // Mappings & templates
  void add_mapping(const Mapping &m) { mappings_.push_back(m); }
  void add_template(const FrameTemplate &t) { templates_.push_back(t); }

  // Variable access (scaled). Returns NAN if unknown.
  float get_var(const std::string &name) const;

  // Enqueue an inject job by template name and optional overrides.
  bool send_template(const std::string &template_name,
                     const std::unordered_map<std::string, std::string> &overrides);

  // Stats accessors (for HA sensors)
  uint32_t get_frames_forwarded(Direction dir) const { return frames_forwarded_[static_cast<int>(dir)]; }
  uint32_t get_frames_parsed() const { return frames_parsed_; }
  uint32_t get_checksum_errors() const { return checksum_errors_; }
  uint32_t get_parity_errors(Direction dir) const { return parity_errors_[static_cast<int>(dir)]; }

  // Component hooks
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  // UARTs
  uart::UARTComponent *uart_a_{nullptr}; // side A
  uart::UARTComponent *uart_b_{nullptr}; // side B

  // RX state per direction
  struct RxState {
    std::vector<uint8_t> buffer;
    std::vector<uint16_t> interbyte_us; // optional timing capture
    uint32_t last_rx_ms{0};
    bool     frame_ready{false};
  } rx_[2];

  // TX & inject
  std::vector<InjectJob> inject_queue_; // simple FIFO v1
  bool inject_enabled_{false};          // can be toggled via HA switch (later)

  // Config
  uint32_t silence_ms_{DEFAULT_SILENCE_MS};
  uint32_t pre_listen_ms_{DEFAULT_PRE_LISTEN_MS};
  uint16_t max_frame_{DEFAULT_MAX_FRAME};

  // Mappings/templates
  std::vector<Mapping> mappings_;
  std::vector<FrameTemplate> templates_;

  // Stored variables
  std::unordered_map<std::string, StoredVar> vars_;

  // Stats
  uint32_t frames_forwarded_[2]{0,0};
  uint32_t frames_parsed_{0};
  uint32_t checksum_errors_{0};
  uint32_t parity_errors_[2]{0,0};

  // ---- Internal helpers (implemented in .cpp) ------------------------------
  void read_uart_(Direction dir);
  void on_silence_expired_(Direction dir);
  void forward_frame_(Direction dir, const std::vector<uint8_t> &frame);

  void parse_and_store_(const std::vector<uint8_t> &frame);
  bool selector_match_(const FrameSelector &sel, const std::vector<uint8_t> &frame) const;

  // Field decoding/encoding
  bool decode_field_(const FieldDescriptor &fd,
                     const std::vector<uint8_t> &frame,
                     std::string &out_value_str, float &out_value_scaled,
                     std::vector<uint8_t> &out_raw) const;

  bool encode_template_field_(const TemplateField &tf, const std::string &value_str,
                              std::vector<uint8_t> &out_bytes) const;

  // LRC (8-bit two’s complement of sum)
  static uint8_t calc_lrc_(const uint8_t *data, size_t len);

  // Build frame from template + overrides or stored vars
  bool build_frame_from_template_(const FrameTemplate &tp,
                                  const std::unordered_map<std::string, std::string> &overrides,
                                  std::vector<uint8_t> &out_frame);

  // Inject scheduler
  void process_inject_queue_();

  // Bus-idle detection
  bool bus_idle_(Direction dir) const;

  // Utility
  static uart::UARTComponent *rx_uart_(Direction dir, uart::UARTComponent *a, uart::UARTComponent *b) {
    return (dir == Direction::A_TO_B) ? a : b;
  }
  static uart::UARTComponent *tx_uart_(Direction dir, uart::UARTComponent *a, uart::UARTComponent *b) {
    return (dir == Direction::A_TO_B) ? b : a;
  }
};

}  // namespace ghostuart
}  // namespace esphome
