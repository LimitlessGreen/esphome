import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_RX_ONLY,
    CONF_CO,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_EMPTY,
    UNIT_PARTS_PER_MILLION,
    ICON_CHEMICAL_WEAPON,
)

DEPENDENCIES = ["uart"]

ze07_ns = cg.esphome_ns.namespace("ze07")
ZE07COComponent = ze07_ns.class_("ZE07COComponent", uart.UARTDevice, cg.Component)


def validate_ze07_rx_mode(value):
    if value.get(CONF_RX_ONLY) and CONF_UPDATE_INTERVAL in value:
        # update_interval does not affect anything in rx-only mode, let's warn user about
        # that
        raise cv.Invalid(
            "update_interval has no effect in rx_only mode. Please remove it.",
            path=["update_interval"],
        )
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZE07COComponent),
            cv.Optional(CONF_CO): sensor.sensor_schema(
                UNIT_PARTS_PER_MILLION,
                ICON_CHEMICAL_WEAPON,
                1,
                DEVICE_CLASS_EMPTY,
            ),
            cv.Optional(CONF_RX_ONLY, default=True): cv.boolean,
            cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_minutes,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    validate_ze07_rx_mode,
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

    if CONF_UPDATE_INTERVAL in config:
        cg.add(var.set_update_interval_min(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_rx_mode_only(config[CONF_RX_ONLY]))

    if CONF_CO in config:
        sens = yield sensor.new_sensor(config[CONF_CO])
        cg.add(var.set_co_sensor(sens))
