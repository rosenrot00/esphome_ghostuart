#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "freertos/queue.h"

namespace esphome {
namespace ghostuart {

// Defaults (can be overridden via YAML setters)
constexpr uint32_t DEFAULT_BAUD = 9600;
constexpr uint16_t DEFAULT_MAX_FRAME = 512;
constexpr uint16_t DEFAULT_RX_BUF = 4096;
constexpr uint16_t DEFAULT_TX_BUF = 2048;
constexpr uint32_t DEFAULT_SILENCE_MS = 15;        // 0 = auto (from baud)
constexpr uint32_t DEFAULT_PRE_LISTEN_MS = 3;      // 0 = auto (from baud)
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

// Filter action for completed frames
enum class FilterAction : uint8_t {
  NORMAL = 0,     // log + forward
  DROP,           // neither log nor forward
  FORWARD_ONLY,   // forward but do not log RX
  LOG_ONLY        // log RX but do not forward
};

struct FilterRule {
  Direction direction;              // Direction::A_TO_B or Direction::B_TO_A
  std::vector<uint8_t> prefix;      // Header prefix to match at the beginning of the frame
  FilterAction action;
};

class GhostUARTComponent : public Component {
 public:
  // Wiring
  void set_uart_a(uart::UARTComponent *u) { uart_a_ = u; }
  void set_uart_b(uart::UARTComponent *u) { uart_b_ = u; }

  // Shared baud rate for A and B; used to compute auto timings when silence_ms/pre_listen_ms == 0
  void set_baud(uint32_t baud) { baud_ = baud; recompute_timing_(); }

  // Timing/config (YAML setters – 0 means auto)
  void set_silence_ms(uint32_t ms) { silence_ms_cfg_ = ms; recompute_timing_(); }
  void set_pre_listen_ms(uint32_t ms) { pre_listen_ms_cfg_ = ms; recompute_timing_(); }
  void set_max_frame(uint16_t n) { max_frame_ = n; }

  // Parity: 0=None, 1=Even, 2=Odd
  void set_parity_mode(uint8_t m) { parity_mode_ = m; }

  // IDF driver mode (optional): switch to native ESP-IDF UART driver + event-queue
  void set_use_idf_driver(bool en) { use_idf_driver_ = en; }
  void set_idf_uart_nums(int a, int b) { idf_uart_num_a_ = a; idf_uart_num_b_ = b; }
  void set_idf_rx_bufs(uint32_t a, uint32_t b) { idf_rx_buf_a_ = a; idf_rx_buf_b_ = b; }
  void set_idf_rx_timeout_chars(uint8_t a, uint8_t b) { idf_rx_timeout_chars_a_ = a; idf_rx_timeout_chars_b_ = b; }
  void set_idf_uart_pins_a(int tx, int rx) { idf_tx_pin_a_ = tx; idf_rx_pin_a_ = rx; }
  void set_idf_uart_pins_b(int tx, int rx) { idf_tx_pin_b_ = tx; idf_rx_pin_b_ = rx; }

  // Add a frame filter rule from numeric codes (used by YAML/codegen)
  void add_filter_rule(uint8_t dir_code, const std::vector<uint8_t> &prefix, uint8_t action_code);

  // Debug control
  void set_debug(bool en);

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

  // Component hooks
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // Service RX and close frames on idle (called from the dedicated RX task)
  void rx_task_tick();

  // Query RX task enabled state (used by the static RX task)
  bool rx_task_is_enabled() const { return rx_task_enabled_; }

  // Enable/disable the RX task (e.g., suspend during OTA)
  void set_rx_task_enabled(bool en);

 protected:
  // UARTs (ESPHome path)
  uart::UARTComponent *uart_a_{nullptr}; // side A
  uart::UARTComponent *uart_b_{nullptr}; // side B

  // Runtime config
  uint32_t baud_{DEFAULT_BAUD};
  uint16_t max_frame_{DEFAULT_MAX_FRAME};
  uint32_t silence_ms_cfg_{DEFAULT_SILENCE_MS};       // 0 = auto (from baud)
  uint32_t pre_listen_ms_cfg_{DEFAULT_PRE_LISTEN_MS}; // 0 = auto (from baud)
  uint32_t silence_ms_eff_{DEFAULT_SILENCE_MS};       // effective value after auto calculation
  uint32_t pre_listen_ms_eff_{DEFAULT_PRE_LISTEN_MS};
  uint8_t parity_mode_{1};  // 0=None, 1=Even (default), 2=Odd

  // RX state (per-direction)
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

  // Stored variables & stats
  std::unordered_map<std::string, StoredVar> vars_;
  uint32_t frames_forwarded_[2]{0,0};
  uint32_t frames_parsed_{0};
  uint32_t checksum_errors_{0};

  // Ready (fully delimited) frames produced by the RX task; processed in loop()
  std::vector<std::vector<uint8_t>> ready_frames_[2];

  // Enqueue a finished frame from RX task
  void enqueue_ready_frame_(Direction dir, std::vector<uint8_t> &&frame);

  // Optional per-instance frame filters
  std::vector<FilterRule> filters_;

  // Match filters for a given frame and direction; returns FilterAction::NORMAL if no rule matches.
  FilterAction match_filters_(Direction dir, const std::vector<uint8_t> &frame) const;

  // IDF/native-UART driver configuration & state (optional)
  bool use_idf_driver_{false};                // when true, initialize native IDF uart drivers and use event queues
  int  idf_uart_num_a_{-1};                   // UART port numbers (UART_NUM_0/1/2) for side A
  int  idf_uart_num_b_{-1};                   // UART port numbers for side B
  uint32_t idf_rx_buf_a_{DEFAULT_RX_BUF};     // RX buffer size used with uart_driver_install
  uint32_t idf_rx_buf_b_{DEFAULT_RX_BUF};
  uint8_t  idf_rx_timeout_chars_a_{0};        // idle timeout in chars (0 = disabled)
  uint8_t  idf_rx_timeout_chars_b_{0};
  int idf_tx_pin_a_{-1}, idf_rx_pin_a_{-1};
  int idf_tx_pin_b_{-1}, idf_rx_pin_b_{-1};

  // IDF runtime handles
  QueueHandle_t idf_uart_queue_a_{nullptr};
  QueueHandle_t idf_uart_queue_b_{nullptr};
  bool idf_driver_installed_a_{false};
  bool idf_driver_installed_b_{false};

  // Debug flag
  bool debug_enabled_{false};

  // RX task control
  TaskHandle_t rx_task_handle_{nullptr};
  bool rx_task_enabled_{true};

  // Internals
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

  bool bus_idle_(Direction dir) const;
  void recompute_timing_(); // derive effective timing from configured baud

  // IDF helpers
  bool idf_init_uart_(int uart_num, int tx_pin, int rx_pin,
                      uint32_t rx_buf, uint8_t timeout_chars,
                      QueueHandle_t &out_queue, const char *side_tag);
  void idf_service_events_(int uart_num, QueueHandle_t queue, Direction dir);

  static uart::UARTComponent *rx_uart_(Direction dir, uart::UARTComponent *a, uart::UARTComponent *b) {
    return (dir == Direction::A_TO_B) ? a : b;
  }
  static uart::UARTComponent *tx_uart_(Direction dir, uart::UARTComponent *a, uart::UARTComponent *b) {
    return (dir == Direction::A_TO_B) ? b : a;
  }
};

}  // namespace ghostuart
}  // namespace esphome
