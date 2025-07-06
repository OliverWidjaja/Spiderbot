from pyvesc.VESC.VESC import VESC
import numpy as np
import math, time


motor = VESC(serial_port='COM3')

motor.set_pos(0)
motor.set_pos(0, can_id=119)
time.sleep(1)

x = 0
y = 0

# Motor Command
for i in range(10):
    x += 20
    y += 20
    motor.set_pos(x)
    motor.set_pos(y, can_id=119)

    time.sleep(0.25)

motor.stop_heartbeat()
motor.serial_port.flush()
motor.serial_port.close()
