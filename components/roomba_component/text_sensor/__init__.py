import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID
from .. import RoombaComponent, roomba_component_ns, CONF_ROOMBACOMPONENT_ID

RoombaActivitySensor = roomba_component_ns.class_("RoombaActivitySensor", text_sensor.TextSensor)
CONF_ACTIVITY = "activity"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ROOMBACOMPONENT_ID): cv.use_id(RoombaComponent),
    cv.Optional(CONF_ACTIVITY): text_sensor.text_sensor_schema(RoombaActivitySensor),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ROOMBACOMPONENT_ID])
    if CONF_ACTIVITY in config:
        conf = config[CONF_ACTIVITY]
        var = cg.new_Pvariable(conf[CONF_ID])
        await text_sensor.register_text_sensor(var, conf)
        cg.add(parent.register_sensor(var))