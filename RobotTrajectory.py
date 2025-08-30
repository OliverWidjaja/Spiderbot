from VESCCommands import zero_pos, move_motors
import numpy as np
import time


try:
    # Taut Initial Zero Position
    zero_pos(serial_port='COM3', can_id_2=119)
    time.sleep(1)

    #  Coordinate Values (x,y in mm)
    nest_cable1 = np.array([1000, 4000]) 
    nest_cable2 = np.array([9000, 4000])
    vesc_cable1 = np.array([4000, 2000])
    vesc_cable2 = np.array([6000, 2000])

    # Calculated Cable Change
    cable_length_1 = np.linalg.norm(nest_cable1 - vesc_cable1)
    cable_length_2 = np.linalg.norm(nest_cable2 - vesc_cable2)
    level_cable_length_1 = abs(nest_cable1[0] - vesc_cable1[0])
    level_cable_length_2 = abs(nest_cable2[0] - vesc_cable2[0])
    cable_displacement_1 = cable_length_1 - level_cable_length_1
    cable_displacement_2 = cable_length_2 - level_cable_length_2

    # Move motors using VESCCommands
    move_motors(cable_displacement_1, cable_displacement_2)


except KeyboardInterrupt:
    print("\nInterrupted by user.")
