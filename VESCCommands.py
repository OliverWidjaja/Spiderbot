from pyvesc import VESC
from pyvesc.VESC.messages import SetRPM, SetCurrent, GetValues
import pyvesc
import time
import numpy as np

"""
This script ASSUMES VESC drums are in zero position before running.
"""


def move_motors(length_1_mm, length_2_mm, T, dt=0.1, serial_port='COM3', step_size=10, can_id_2=119, cleanup_serial=True):
    """
    Move two motors by specifying cable length to wind/unwind (in mm) for each motor.
    Args:
        length_1_mm (float): Cable length to move for motor 1 (mm)
        length_2_mm (float): Cable length to move for motor 2 (mm)
        T (float): Total time for the movement (seconds)
        dt (float): Time between steps in seconds
        serial_port (str): Serial port for VESC
        step_size (int): Step size for each increment (in angle, default 10)
        can_id_2 (int): CAN ID for motor 2
        cleanup_serial (bool): Whether to flush/close serial port after
    """
    # VESC Cable Drum spec (mm)
    vesc_cable_drum_diam = 32.5 * 2
    vesc_cable_drum_circum = np.pi * vesc_cable_drum_diam
    # mm_per_deg = vesc_cable_drum_circum / 360
    mm_per_deg = 200 / 360

    # Convert length to angle (deg)
    angle_1 = (length_1_mm / mm_per_deg)
    angle_2 = (length_2_mm / mm_per_deg)
    total_angle_change_1 = angle_1
    total_angle_change_2 = angle_2

    motor = VESC(serial_port=serial_port)
    try:
        angle_1_offset = motor.get_pos()
        angle_2_offset = motor.get_pos(can_id=can_id_2)

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

    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping motors and cleaning up...")
    finally:
        try:
            motor.stop_heartbeat()
        except Exception:
            pass
        if cleanup_serial:
            try:
                motor.serial_port.flush()
            except Exception:
                pass
            try:
                motor.serial_port.close()
            except Exception:
                pass

# Example usage:
if __name__ == "__main__":

    # Test move commands
    move_motors(0, 500, 5, 0.01)
    move_motors(0, -500, 5, 0.01)
