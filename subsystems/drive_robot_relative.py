import commands2
import wpimath.geometry
import subsystems.swerve_drivetrain
import wpimath
import wpimath.units
import phoenix6.swerve
import math

FORWARD_OFFSET = wpimath.units.inchesToMeters(22.0) # inches away from the Coral posts
CORAL_POST_OFFSET = wpimath.units.inchesToMeters(-3.0) # inches offset from center of AprilTag
ONE_INCH = wpimath.units.inchesToMeters(1)
TWO_INCHES = wpimath.units.inchesToMeters(2)
TWO_DEGREES = wpimath.units.degrees(2)

ROBOT_RELATIVE = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

TURBO_SPEED = 1.0
NORMAL_SPEED = 0.5
SLOW_SPEED = 0.25

class DriveRobotRelativeCommand(commands2.Command):
    def __init__(self, 
                 drive_train: subsystems.swerve_drivetrain.SwerveDrivetrain, 
                 offset: wpimath.geometry.Transform2d,
                 speed: float,
                 ):
        self.addRequirements(drive_train)
        self.drive_train = drive_train
        self.offset = offset
        self.speed = speed

        self.forward = 0.0
        self.horizontal = 0.0
        self.rotation = 0.0

        self.start_pose = wpimath.geometry.Pose2d()
        self.end_pose = wpimath.geometry.Pose2d()


    def initialize(self):
        self.start_pose = self.drive_train.get_pose()
        self.end_pose = self.start_pose + self.offset
        
        forward = 0.0
        horizontal = 0.0
        rotation = 0.0

        compare_x = self.offset.X()
        compare_y = self.offset.Y()
        compare_r = self.offset.rotation().degrees()

        # Determine our speed for each direction.
        if compare_x > ONE_INCH:
            forward = self.speed
        elif compare_x < -ONE_INCH:
            forward = -1 * self.speed
        if compare_y > ONE_INCH:
            horizontal = self.speed
        elif compare_y < -ONE_INCH:
            horizontal = -1 * self.speed
        if compare_r > TWO_DEGREES:
            rotation = self.speed
        elif compare_r < -TWO_DEGREES:
            rotation = -1 * self.speed

        self.forward = forward
        self.horizontal = horizontal
        self.rotation = rotation
        
    def execute(self):
        current_pose = self.drive_train.get_pose()
        diff = self.end_pose - current_pose

        # Check each direction separately to know if we're within range or need to change direction
        # forward
        if abs(diff.X()) < ONE_INCH:
            self.forward = 0.0

        # horizontal
        if abs(diff.Y()) < ONE_INCH:
            self.horizontal = 0.0

        # rotation
        r = diff.rotation().degrees()
        if abs(r) < TWO_DEGREES:
            self.rotation = 0.0
        
        self.drive_cmd = self.drive_train.drive_cmd(self.forward, self.horizontal, self.rotation)
        self.drive_cmd.execute()

    def isFinished(self):
        current_pose = self.drive_train.get_pose()
        diff = self.end_pose - current_pose

        # Stop if all directions are within range
        return abs(diff.X()) < ONE_INCH and abs(diff.Y()) < ONE_INCH and abs(diff.rotation().degrees()) < TWO_DEGREES
    
    def end(self, interrupted):
        self.drive_train.drive_cmd(0, 0, 0).execute()

def drive_command(drive_train: subsystems.swerve_drivetrain.SwerveDrivetrain, offset: float, speed: float, rotation: wpimath.units.radians):
    pose = wpimath.geometry.Transform2d(offset, 0.0, rotation)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_sideways_command(drive_train: subsystems.swerve_drivetrain.SwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Transform2d(0.0, offset, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)
