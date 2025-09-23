from pyvesc import VESC
from pyvesc.VESC.messages import SetRPM, SetCurrent, GetValues
import time


# TO DO: Hard-code maximum cable length spooled (~1600mm)
def move_motors(motor: VESC, length_1_mm, length_2_mm, T, dt=0.01, can_id_2=119):
    # VESC Cable Drum spec (mm)
    mm_per_deg = 200 / 360

    # Convert length to angle (deg)
    angle_1 = (length_1_mm / mm_per_deg)
    angle_2 = (length_2_mm / mm_per_deg)

    while True:
        try:
            angle_1_offset = motor.get_pos()
            angle_2_offset = motor.get_pos(can_id=can_id_2)
            break
        except Exception as e:
            print(f"Error getting initial motor positions: {e}")
            print("Retrying ...")
            time.sleep(0.1)

    curr_angle_1 = angle_1_offset
    curr_angle_2 = angle_2_offset


    num_of_steps = int(T/dt)
    step_size_1 = angle_1/num_of_steps
    step_size_2 = angle_2/num_of_steps

    for _ in range(num_of_steps):
        curr_angle_1 += step_size_1
        curr_angle_1 = curr_angle_1 % 360
        motor.set_pos(curr_angle_1)

        curr_angle_2 += step_size_2
        curr_angle_2 = curr_angle_2 % 360
        motor.set_pos(curr_angle_2, can_id=can_id_2)

        time.sleep(dt)

if __name__ == "__main__":
    motor = VESC(serial_port='COM3')

    move_motors(motor, -100, -100, 5)
    move_motors(motor, 100, 100, 5)
