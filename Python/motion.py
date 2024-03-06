import time
import json
from machine import Pin
from breakout_paa5100 import BreakoutPAA5100 as FlowSensor

flo = FlowSensor()
flo.set_rotation(FlowSensor.DEGREES_0)
led = Pin(25,Pin.OUT)

while True:
    delta = flo.get_motion(timeout=0.1)
    if delta is not None:
        x = delta[0]
        y = delta[1]
        result = { "x":x, "y":y }
        led.value(1)
        print(json.dumps(result))
    time.sleep(0.1)
    led.value(0)