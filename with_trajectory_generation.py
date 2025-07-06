import numpy as np
import math, time
import with_motor_commands

# This sequence will run after the drone has flown and landed on the nest with cable attached.
# Actual cable unspooling mechanism of the drone flying mechanism has yet to be completed.

# Taut Initial Zero Position
with_motor_commands.zero_pos()
time.sleep(2)

# VESC Cable Drum spec
vesc_cable_drum = 33.25 # (mm)
vesc_cable_drum_circum = np.pi * 33.25
vesc_cable_drum_1deg = (np.pi * 33.25) / 360

# Angles Initialization
curr_angle_1 = 0 
curr_angle_2 = 0
motor_1_done = False
motor_2_done = False
counter_1 = 0
counter_2 = 0

# Timestep (s)
dt = 0.01 

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
motor_displacement_1 = cable_length_1 - level_cable_length_1
motor_displacement_2 = cable_length_2 - level_cable_length_2

# Calculated Angle Change
total_angle_change_1 = motor_displacement_1 / vesc_cable_drum_1deg
total_angle_change_2 = motor_displacement_2 / vesc_cable_drum_1deg
total_angle_change_1 = math.ceil(total_angle_change_1)
total_angle_change_2 = math.ceil(total_angle_change_2)

# Motor Command
for i in range(max(total_angle_change_1, total_angle_change_2) // 10):
    if counter_1 < total_angle_change_1:
        curr_angle_1 += 10
        counter_1 += 10
        if curr_angle_1 > 360:
            curr_angle_1 -= 360
        with_motor_commands.set_pos_1(curr_angle_1)

    if counter_2 < total_angle_change_2:
        curr_angle_2 += 10
        counter_2 += 10
        if curr_angle_2 > 360:
            curr_angle_2 -= 360
        with_motor_commands.set_pos_2(curr_angle_2)

    time.sleep(dt)
