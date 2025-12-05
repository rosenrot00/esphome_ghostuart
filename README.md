# ESPHome GhostUART

ESPHome GhostUART is a two-UART man-in-the-middle component for ESP32 that lets you intercept, analyze and selectively forward serial communication between two devices (side A ↔ side B).  
It is designed for reverse-engineering proprietary UART protocols (for example Daikin heat pumps) and exposing selected values to Home Assistant via ESPHome.

## Features

- Works as a transparent UART↔UART bridge between two devices
- Optional ESP-IDF hardware UART driver usage for robust timing
- Configurable filters per direction (A→B / B→A):
  - drop frames
  - log frames
  - forward frames only (suppress all others)
- Mapping of arbitrary frame bytes into named variables
- Access mapped variables from template sensors / binary sensors using `get_var(name)`
- Optional debug logging with change-only logging per prefix

---

## Example

Add GhostUART as an external ESPHome component.

```yaml
esphome:
  name: esphome-web-df7910
  friendly_name: ghostuart
  name_add_mac_suffix: false

esp32:
  variant: esp32s3
  framework:
    type: esp-idf

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

logger:
api:
ota:
  - platform: esphome

external_components:
  - source:
      type: git
      url: https://github.com/rosenrot00/esphome_ghostuart
    components: [ghostuart]
    refresh: always

ghostuart:
  id: ghost

  # Use ESP-IDF UART driver (recommended on ESP32)
  use_idf_driver: true

  # Physical UART indices to use (ESP-IDF numbering)
  idf_uart_nums: [0, 1]

  # Side A: TX/RX pins for UART A (index 0)
  # Side B: TX/RX pins for UART B (index 1)
  idf_uart_pins_a: [4, 5]   # TX, RX
  idf_uart_pins_b: [6, 7]   # TX, RX

  # RX buffer sizes per UART in bytes
  idf_rx_bufs: [1024, 1024]

  # UART line settings
  parity: EVEN              # NONE | EVEN | ODD
  baud: 9600

  # RX timeout in "character times" (per UART)
  # 10–12 chars are good starting values at 9600 8E1
  idf_rx_timeout_chars: [2, 2]

  # Maximum frame size
  max_frame: 512

  # Silence-based detection (ignored in IDF mode, hardware idle is used)
  silence_ms: 0

  # Start listening shortly before forwarding
  pre_listen_ms: 1

  # Enable verbose debug logging
  debug: true

  # Frame filters (directional)
  filters:
    # Drop any A→B frames starting with 0x00
    - direction: A_TO_B
      prefix: [0x00]
      action: drop

    # Only log A→B frames with prefix 40 00 10, but still forward them
    - direction: A_TO_B
      prefix: [0x40, 0x00, 0x10]
      action: log_only
      log_change_only: true

    # Forward only B→A frames starting with 00 F0 and log only on change
    - direction: B_TO_A
      prefix: [0x00, 0xF0]
      action: forward_only
      log_change_only: true

  # Mapping of specific frames into named variables
  mappings:

    # Example: decode multiple UINT8 values from B→A frames starting with 00 00 10
    - name: daikin_values_type0010
      direction: B_TO_A
      prefix: [0x00, 0x00, 0x10]
      fields:
        - name: temp0010_3
          offset: 3
          length: 1
          format: UINT8
          scale: 1
        - name: temp0010_4
          offset: 4
          length: 1
          format: UINT8
          scale: 1

    # Example: decode a signed 16-bit big-endian value scaled by 1/256
    - name: daikin_values_type0011
      direction: B_TO_A
      prefix: [0x00, 0x00, 0x11]
      fields:
        - name: temp0011_4
          offset: 4
          length: 2
          format: INT16_BE
          scale: 0.00390625    # 1 / 256

binary_sensor:
  - platform: template
    name: "Shower water active"
    id: daikin_mode_flag_bin
    device_class: running
    lambda: |-
      float v = id(ghost).get_var("mode_flag");
      if (isnan(v)) {
        // No value received yet
        return {};
      }
      // 1 → true (ON), 0 → false (OFF)
      return v > 0.5;

  - platform: template
    name: "Heating running"
    id: daikin_running_flag_bin
    device_class: running
    lambda: |-
      float v = id(ghost).get_var("running_flag");
      if (isnan(v)) {
        return {};
      }
      return v > 0.5;

sensor:
  - platform: template
    name: "Flow temperature"
    id: daikin_flow_temp
    unit_of_measurement: "°C"
    accuracy_decimals: 1
    lambda: |-
      float v = id(ghost).get_var("temp4011_4");
      if (isnan(v)) return NAN;
      return v;

  - platform: template
    name: "Room temperature"
    id: daikin_room_temp
    unit_of_measurement: "°C"
    accuracy_decimals: 1
    lambda: |-
      float v = id(ghost).get_var("temp0011_4");
      if (isnan(v)) return NAN;
      return v;

  - platform: template
    name: "Shower water temperature"
    id: daikin_shower_temp
    unit_of_measurement: "°C"
    accuracy_decimals: 1
    lambda: |-
      float v = id(ghost).get_var("temp4011_7");
      if (isnan(v)) return NAN;
      return v;
