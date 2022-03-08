import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID

CODEOWNERS = ["@proto"]

DEPENDENCIES = ["i2c"]

custom_ezo_ns = cg.esphome_ns.namespace("ezo")

CustomEZOSensor = custom_ezo_ns.class_(
    "CustomEZOSensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    sensor.SENSOR_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(CustomEZOSensor),
        }
    )
    .extend(cv.polling_component_schema("0.1s"))
    .extend(i2c.i2c_device_schema(None))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    await i2c.register_i2c_device(var, config)