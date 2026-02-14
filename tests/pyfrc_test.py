'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

# Run the default tests, takes ~20 seconds
from pyfrc.tests import * # type: ignore

import wpimath.geometry
import subsystems.limelight_positions

# Then run our own tests

def float_equals(a: float, b: float) -> bool:
    return abs(a - b) < 0.001

def test_is_pose2d_zero():
    # None
    test : wpimath.geometry.Pose2d
    test = None # type: ignore
    output = subsystems.limelight_positions.is_pose2d_zero(test)
    assert(output == False)

    # Zero
    test = wpimath.geometry.Pose2d()
    output = subsystems.limelight_positions.is_pose2d_zero(test)
    assert(output == True)

    # Non-zero
    test = wpimath.geometry.Pose2d(10, 11, wpimath.geometry.Rotation2d.fromDegrees(12))
    output = subsystems.limelight_positions.is_pose2d_zero(test)
    assert(output == False)
test_is_pose2d_zero()

def test_pose2d_from_targetpose():
    # None
    test = None
    output = subsystems.limelight_positions.pose2d_from_targetpose(test) # type: ignore
    assert(subsystems.limelight_positions.is_pose2d_zero(output))

    # Empty
    test = []
    output = subsystems.limelight_positions.pose2d_from_targetpose(test)
    assert(subsystems.limelight_positions.is_pose2d_zero(output))

    # Normal
    test = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    output = subsystems.limelight_positions.pose2d_from_targetpose(test)
    assert(float_equals(output.X(), 0.3))
    assert(float_equals(output.Y(), 0.1))
    assert(float_equals(output.rotation().degrees(), 0.5))
test_pose2d_from_targetpose()

def test_pose2d_from_botpose():
    # None
    test = None
    output = subsystems.limelight_positions.pose2d_from_botpose(test) # type: ignore
    assert(subsystems.limelight_positions.is_pose2d_zero(output))

    # Empty
    test = []
    output = subsystems.limelight_positions.pose2d_from_botpose(test)
    assert(subsystems.limelight_positions.is_pose2d_zero(output))

    # Normal
    test = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    output = subsystems.limelight_positions.pose2d_from_botpose(test)
    assert(float_equals(output.X(), 0.1))
    assert(float_equals(output.Y(), 0.2))
    assert(float_equals(output.rotation().degrees(), 0.6))
test_pose2d_from_botpose()

def test_correct_target_pose():
    # None
    test = None
    output = subsystems.limelight_positions.correct_target_pose(test) # type: ignore
    assert(output is None)

    # Zero
    test = wpimath.geometry.Pose2d()
    output = subsystems.limelight_positions.correct_target_pose(test)
    assert(subsystems.limelight_positions.is_pose2d_zero(output))

    # Correctly negates the rotation and horizontal
    test = wpimath.geometry.Pose2d(2, 3, wpimath.geometry.Rotation2d.fromDegrees(4))
    output = subsystems.limelight_positions.correct_target_pose(test)
    assert(float_equals(output.X(),2))
    assert(float_equals(output.Y(),-3))
    assert(float_equals(output.rotation().degrees(), -4))
test_correct_target_pose()