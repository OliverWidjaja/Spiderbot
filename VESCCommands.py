from pyvesc.VESC.VESC import VESC
import time
import numpy as np


def zero_pos(serial_port='COM3', can_id_2=119, cleanup_serial=True):
    motor = VESC(serial_port=serial_port)
    try:
        motor.set_pos(0)
        motor.set_pos(0, can_id=can_id_2)
        time.sleep(1)
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

def move_motors(length_1_mm, length_2_mm, dt=0.1, serial_port='COM3', step_size=10, can_id_2=119, cleanup_serial=True):
    """
    Move two motors by specifying cable length to wind/unwind (in mm) for each motor.
    Args:
        length_1_mm (float): Cable length to move for motor 1 (mm)
        length_2_mm (float): Cable length to move for motor 2 (mm)
        dt (float): Time between steps in seconds
        serial_port (str): Serial port for VESC
        step_size (int): Step size for each increment (in angle, default 10)
        can_id_2 (int): CAN ID for motor 2
        cleanup_serial (bool): Whether to flush/close serial port after
    """
    # VESC Cable Drum spec (mm)
    vesc_cable_drum = 33.25
    vesc_cable_drum_circum = np.pi * vesc_cable_drum
    deg_per_mm = vesc_cable_drum_circum / 360

    # Convert length to angle (deg)
    angle_1 = (length_1_mm / deg_per_mm)
    angle_2 = (length_2_mm / deg_per_mm)
    total_angle_change_1 = int(round(angle_1))
    total_angle_change_2 = int(round(angle_2))

    motor = VESC(serial_port=serial_port)
    try:
        curr_angle_1 = 0
        curr_angle_2 = 0
        counter_1 = 0
        counter_2 = 0

        step_1 = step_size if total_angle_change_1 >= 0 else -step_size
        step_2 = step_size if total_angle_change_2 >= 0 else -step_size
        target_1 = abs(total_angle_change_1)
        target_2 = abs(total_angle_change_2)
        num_steps = max(target_1, target_2) // step_size

        for _ in range(num_steps):
            if abs(counter_1) < target_1:
                curr_angle_1 += step_1
                counter_1 += step_1
                curr_angle_1 = curr_angle_1 % 360
                motor.set_pos(curr_angle_1)

            if abs(counter_2) < target_2:
                curr_angle_2 += step_2
                counter_2 += step_2
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
    # Zero position command
    zero_pos()

    # Test move commands
    move_motors(5, 0, 0.2)
    move_motors(-5, 0, 0.2)
    move_motors(0, 5, 0.2)
    move_motors(0, -5, 0.2)
