import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, switch, time, output, binary_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["sensor", "switch", "time", "output", "binary_sensor"]

esphome_hotcirc_ns = cg.esphome_ns.namespace("esphome_hotcirc")
HotWaterController = esphome_hotcirc_ns.class_("HotWaterController", cg.Component)

CONF_OUTLET_SENSOR = "outlet_sensor"
CONF_RETURN_SENSOR = "return_sensor"
CONF_PUMP_SWITCH = "pump_switch"
CONF_TIME_SOURCE = "time_source"
CONF_BUTTON = "button"
CONF_LED_GREEN = "led_green"
CONF_LED_YELLOW = "led_yellow"
CONF_OUTLET_RISE = "outlet_rise"
CONF_RETURN_RISE = "return_rise"
CONF_DISINFECTION_TEMP_RISE = "disinfection_temp_rise"
CONF_MIN_RETURN_TEMP = "min_return_temp"
CONF_PUMP_FLOW_RATE = "pump_flow_rate"
CONF_ANTI_STAGNATION_INTERVAL = "anti_stagnation_interval"
CONF_ANTI_STAGNATION_RUNTIME = "anti_stagnation_runtime"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HotWaterController),
    cv.Required(CONF_OUTLET_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_RETURN_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_PUMP_SWITCH): cv.use_id(switch.Switch),
    cv.Required(CONF_TIME_SOURCE): cv.use_id(time.RealTimeClock),
    cv.Optional(CONF_BUTTON): cv.use_id(binary_sensor.BinarySensor),
    cv.Optional(CONF_LED_GREEN): cv.use_id(output.BinaryOutput),
    cv.Optional(CONF_LED_YELLOW): cv.use_id(output.BinaryOutput),
    cv.Optional(CONF_OUTLET_RISE, default=1.5): cv.float_range(min=0.1, max=5.0),
    cv.Optional(CONF_RETURN_RISE, default=1.5): cv.float_range(min=0.1, max=10.0),
    cv.Optional(CONF_DISINFECTION_TEMP_RISE, default=10.0): cv.float_range(min=5.0, max=20.0),
    cv.Optional(CONF_MIN_RETURN_TEMP, default=30.0): cv.float_range(min=20.0, max=45.0),
    cv.Optional(CONF_PUMP_FLOW_RATE, default=3.0): cv.float_range(min=0.5, max=50.0),
    cv.Optional(CONF_ANTI_STAGNATION_INTERVAL, default=172800): cv.uint32_t,  # 48 hours in seconds
    cv.Optional(CONF_ANTI_STAGNATION_RUNTIME, default=15): cv.uint32_t,  # 15 seconds
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    outlet = await cg.get_variable(config[CONF_OUTLET_SENSOR])
    ret = await cg.get_variable(config[CONF_RETURN_SENSOR])
    pump = await cg.get_variable(config[CONF_PUMP_SWITCH])
    clock = await cg.get_variable(config[CONF_TIME_SOURCE])

    cg.add(var.set_outlet_sensor(outlet))
    cg.add(var.set_return_sensor(ret))
    cg.add(var.set_pump_switch(pump))
    cg.add(var.set_time_source(clock))

    if CONF_BUTTON in config:
        btn = await cg.get_variable(config[CONF_BUTTON])
        cg.add(var.set_button(btn))
    if CONF_LED_GREEN in config:
        led_g = await cg.get_variable(config[CONF_LED_GREEN])
        cg.add(var.set_led_green(led_g))
    if CONF_LED_YELLOW in config:
        led_y = await cg.get_variable(config[CONF_LED_YELLOW])
        cg.add(var.set_led_yellow(led_y))

    cg.add(var.set_thresholds(
        config[CONF_OUTLET_RISE], 
        config[CONF_RETURN_RISE],
        config[CONF_DISINFECTION_TEMP_RISE],
        config[CONF_MIN_RETURN_TEMP]
    ))
    
    cg.add(var.set_pump_flow_rate(config[CONF_PUMP_FLOW_RATE]))
    cg.add(var.set_anti_stagnation_interval(config[CONF_ANTI_STAGNATION_INTERVAL]))
    cg.add(var.set_anti_stagnation_runtime(config[CONF_ANTI_STAGNATION_RUNTIME]))
