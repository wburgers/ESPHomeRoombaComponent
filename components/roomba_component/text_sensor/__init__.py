import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID
from .. import RoombaComponent, roomba_component_ns, CONF_ROOMBACOMPONENT_ID

RoombaActivitySensor = roomba_component_ns.class_("RoombaActivitySensor", text_sensor.TextSensor)
RoombaChargingStateTextSensor = roomba_component_ns.class_("RoombaChargingStateTextSensor", text_sensor.TextSensor)

CONF_ACTIVITY = "activity"
CONF_CHARGING_STATE = "charging_state"

TEXT_SENSORS = {
    CONF_ACTIVITY: RoombaActivitySensor,
    CONF_CHARGING_STATE: RoombaChargingStateTextSensor,
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ROOMBACOMPONENT_ID): cv.use_id(RoombaComponent),
}).extend({
    cv.Optional(key): text_sensor.text_sensor_schema(cls)
    for key, cls in TEXT_SENSORS.items()
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ROOMBACOMPONENT_ID])
    for key, cls in TEXT_SENSORS.items():
        if key in config:
            conf = config[key]
            var = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(var, conf)
            cg.add(parent.register_sensor(var))