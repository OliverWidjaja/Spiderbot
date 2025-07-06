from pyvesc.VESC.VESC import VESC
import time

serial_port = 'COM3'

# Working Set Commands w/ CAN Forwarding
#motor.set_duty_cycle(.02, can_id=12)
#motor.set_current(0.5)
#motor.set_pos(180, can_id=12)
#motor.set_rpm(100)
#motor.set_rpm(50)

def basic_pos():
    with VESC(serial_port=serial_port) as motor:
        motor.set_pos(0)
        motor.set_pos(0, can_id=12)
        time.sleep(2)

        motor.set_pos(320)
        motor.set_pos(320, can_id=12)
        time.sleep(2)

def zero_pos():
    with VESC(serial_port=serial_port) as motor:
        motor.set_pos(0)
        motor.set_pos(0, can_id=12)

def set_pos_1(angle):
    with VESC(serial_port=serial_port) as motor:
        motor.set_pos(angle)

def set_pos_2(angle):
    with VESC(serial_port=serial_port) as motor:
        motor.set_pos(angle, can_id=12)


if __name__ == '__main__':
    basic_pos()
