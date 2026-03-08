import wpimath.units
import math

encoder_offset_front_left = -0.010986
encoder_offset_front_right = -0.001221
encoder_offset_back_left = -0.006348
encoder_offset_back_right = 0.012939

speed: wpimath.units.meters = 4.0
angular_rate: wpimath.units.radians_per_second = 0.75
drive_gear_ratio = 6.746
turn_gear_ratio = 21.42857
wheel_radius: wpimath.units.meters = wpimath.units.inchesToMeters(2)
wheel_circumference: wpimath.units.meters = wheel_radius * 2 * math.pi

turn_p = 1.0 #25.0
turn_i = 0.0
turn_d = 0.1
turn_s = 0.1
turn_v = 0.0 #1.66
turn_a = 0.0

drive_p = 0.1
drive_i = 0.0
drive_d = 0.0
drive_s = 0.0
drive_v = 0.124
