import automodes
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d

def float_equals(a: float, b: float) -> bool:
    return abs(a - b) < 0.001

def test_mirror_position():
    maxX = wpimath.units.inchesToMeters(650.12)
    maxY = wpimath.units.inchesToMeters(316.64)

    output = automodes.mirror_position(Pose2d(0, 0, 0))
    assert(float_equals(output.Y(), maxY))
    assert(float_equals(output.X(), maxX))
    assert(float_equals(output.rotation().degrees(), 180))

    output = automodes.mirror_position(Pose2d(5, 5, 0))
    assert(float_equals(output.Y(), maxY - 5))
    assert(float_equals(output.X(), maxX - 5))
    assert(float_equals(output.rotation().degrees(), 180))

    output = automodes.mirror_position(Pose2d(maxX, maxY, Rotation2d.fromDegrees(180)))
    assert(float_equals(output.Y(), 0))
    assert(float_equals(output.X(), 0))
    assert(float_equals(output.rotation().degrees(), 360))

    output = automodes.mirror_position(Pose2d(maxX, maxY, Rotation2d.fromDegrees(45)))
    assert(float_equals(output.Y(), 0))
    assert(float_equals(output.X(), 0))
    assert(float_equals(output.rotation().degrees(), 225))

    

def run_tests():
    test_mirror_position()

if __name__ == "__main__":
    run_tests()
