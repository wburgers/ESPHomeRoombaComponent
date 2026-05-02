import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
	CONF_ID,
	CONF_VOLTAGE,
	UNIT_VOLT,
	DEVICE_CLASS_VOLTAGE,
	CONF_CURRENT,
	UNIT_AMPERE,
	DEVICE_CLASS_CURRENT,
	STATE_CLASS_MEASUREMENT,
)

from .. import roomba_component_ns, RoombaComponent, CONF_ROOMBACOMPONENT_ID

DEPENDENCIES = ["roomba_component"]

RoombaComponentSensor = roomba_component_ns.class_("RoombaComponentSensor", sensor.Sensor, cg.PollingComponent)

CONFIG_SCHEMA = (
	cv.Schema(
		{
			cv.GenerateID(): cv.declare_id(RoombaComponentSensor),
			cv.GenerateID(CONF_ROOMBACOMPONENT_ID): cv.use_id(RoombaComponent),
			cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
				unit_of_measurement=UNIT_VOLT,
				icon="mdi:sine-wave",
				accuracy_decimals=2,
				device_class=DEVICE_CLASS_VOLTAGE,
				state_class=STATE_CLASS_MEASUREMENT,
			),
			cv.Optional(CONF_CURRENT): sensor.sensor_schema(
				unit_of_measurement=UNIT_AMPERE,
				icon="mdi:lightning-bolt",
				accuracy_decimals=3,
				device_class=DEVICE_CLASS_CURRENT,
				state_class=STATE_CLASS_MEASUREMENT,
			),
		}
	)
	.extend(cv.polling_component_schema("10s"))
)

async def to_code(config):
	var = cg.new_Pvariable(config[CONF_ID])
	await cg.register_parented(var, config[CONF_ROOMBACOMPONENT_ID])
	await cg.register_component(var, config)

	if CONF_VOLTAGE in config:
		sens = await sensor.new_sensor(config[CONF_VOLTAGE])
		cg.add(var.set_voltage_sensor(sens))

	if CONF_CURRENT in config:
		sens = await sensor.new_sensor(config[CONF_CURRENT])
		cg.add(var.set_current_sensor(sens))