import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_BATTERY_LEVEL, CONF_VOLTAGE, CONF_TEMPERATURE
from .. import RoombaComponent, roomba_component_ns, CONF_ROOMBACOMPONENT_ID

# Define C++ classes
RoombaChargeSensor = roomba_component_ns.class_("RoombaChargeSensor", sensor.Sensor)
RoombaCapacitySensor = roomba_component_ns.class_("RoombaCapacitySensor", sensor.Sensor)
RoombaVoltageSensor = roomba_component_ns.class_("RoombaVoltageSensor", sensor.Sensor)
RoombaBatteryTempSensor = roomba_component_ns.class_("RoombaBatteryTempSensor", sensor.Sensor)
RoombaChargingStateSensor = roomba_component_ns.class_("RoombaChargingStateSensor", sensor.Sensor)

CONF_BATTERY_CAPACITY = "battery_capacity"
CONF_CHARGING_STATE = "charging_state"

# Explicit mapping with hardcoded strings to prevent "string value is None"
SENSORS = {
    CONF_BATTERY_LEVEL: (RoombaChargeSensor, "%", "battery", 0),
    CONF_BATTERY_CAPACITY: (RoombaCapacitySensor, "mAh", "", 0),
    CONF_VOLTAGE: (RoombaVoltageSensor, "V", "voltage", 2),
    CONF_TEMPERATURE: (RoombaBatteryTempSensor, "°C", "temperature", 0),
    CONF_CHARGING_STATE: (RoombaChargingStateSensor, "", "", 0),
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ROOMBACOMPONENT_ID): cv.use_id(RoombaComponent),
}).extend({
    cv.Optional(key): sensor.sensor_schema(
        cls,
        unit_of_measurement=u,
        device_class=d if d else cv.UNDEFINED,
        state_class="measurement",
        accuracy_decimals=a,
    ) for key, (cls, u, d, a) in SENSORS.items()
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ROOMBACOMPONENT_ID])
    for key, (cls, _, _, _) in SENSORS.items():
        if key in config:
            conf = config[key]
            var = cg.new_Pvariable(conf[CONF_ID])
            await sensor.register_sensor(var, conf)
            cg.add(parent.register_sensor(var))