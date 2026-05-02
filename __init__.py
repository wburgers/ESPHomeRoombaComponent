import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

roomba_component_ns = cg.esphome_ns.namespace("roomba_component")
RoombaComponent = roomba_component_ns.class_(
    "RoombaComponent", cg.Component, uart.UARTDevice
)

CONF_ROOMBACOMPONENT_ID = "roombacomponent_id"

CONFIG_SCHEMA = (
	cv.Schema(
		{
			cv.GenerateID(): cv.declare_id(RoombaComponent),
		}
	)
	.extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
	var = cg.new_Pvariable(config[CONF_ID])
	await cg.register_component(var, config)
	await uart.register_uart_device(var, config)