import wpimath.units
import math

encoder_offset_front_left = 0.0
encoder_offset_front_right = 0.0
encoder_offset_back_left = 0.0
encoder_offset_back_right = 0.0

drive_gear_ratio = 6.746
turn_gear_ratio = 21.42857
wheel_radius = wpimath.units.inchesToMeters(2)
wheel_circumference = wheel_radius * 2 * math.pi

turn_p = 0
turn_i = 0
turn_d = 0

drive_p = 0
drive_i = 0
drive_d = 0
