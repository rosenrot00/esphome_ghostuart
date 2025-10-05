import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

AUTO_LOAD = ["uart"]

ghostuart_ns = cg.esphome_ns.namespace("ghostuart")
GhostUARTComponent = ghostuart_ns.class_("GhostUARTComponent", cg.Component)

CONF_UART_A = "uart_a"
CONF_UART_B = "uart_b"
CONF_MAX_FRAME = "max_frame"
CONF_SILENCE_MS = "silence_ms"
CONF_PRE_LISTEN_MS = "pre_listen_ms"
CONF_MAX_RETRIES = "max_retries"
CONF_DEBUG = "debug"
CONF_BAUD = "baud"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(GhostUARTComponent),
        cv.Required(CONF_UART_A): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_UART_B): cv.use_id(uart.UARTComponent),

        # Frame and timing limits. 0 means "auto" (computed from configured baud).
        cv.Optional(CONF_MAX_FRAME, default=512): cv.int_range(min=32, max=4096),
        cv.Optional(CONF_SILENCE_MS, default=15): cv.int_range(min=0, max=10000),
        cv.Optional(CONF_PRE_LISTEN_MS, default=3): cv.int_range(min=0, max=10000),
        cv.Optional(CONF_MAX_RETRIES, default=2): cv.int_range(min=0, max=10),
        cv.Optional(CONF_DEBUG, default=False): cv.boolean,
        cv.Optional(CONF_BAUD, default=9600): cv.int_range(min=300, max=1000000),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Link both UARTs (side A and side B)
    uart_a = await cg.get_variable(config[CONF_UART_A])
    uart_b = await cg.get_variable(config[CONF_UART_B])
    cg.add(var.set_uart_a(uart_a))
    cg.add(var.set_uart_b(uart_b))
    cg.add(var.set_baud(config[CONF_BAUD]))

    # Frame limit
    cg.add(var.set_max_frame(config[CONF_MAX_FRAME]))

    # Silence and pre-listen timing (0 = auto -> computed from baud)
    cg.add(var.set_silence_ms(config[CONF_SILENCE_MS]))
    cg.add(var.set_pre_listen_ms(config[CONF_PRE_LISTEN_MS]))

    # Debug initial state (can be toggled at runtime via set_debug)
    cg.add(var.set_debug(config[CONF_DEBUG]))

    # Note: max_retries is currently managed internally; add setter if you want to expose it.
