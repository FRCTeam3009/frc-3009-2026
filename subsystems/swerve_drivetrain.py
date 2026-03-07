import wpimath.geometry
import commands2.command
import ntcore
import phoenix6
import can_ids
import wpimath.kinematics
import wpimath.units
import subsystems.swerve_module

class SwerveDrivetrain(commands2.Subsystem):
    def __init__(self):
        self.pigeon = phoenix6.hardware.Pigeon2(can_ids.pigeon)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(10), wpimath.units.inchesToMeters(10)),
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(10), wpimath.units.inchesToMeters(-10)),
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(-10), wpimath.units.inchesToMeters(10)),
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(-10), wpimath.units.inchesToMeters(-10)),
        )
        self.swerve_states = self.kinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds())

        self.swerve_positions = (
            wpimath.kinematics.SwerveModulePosition(),
            wpimath.kinematics.SwerveModulePosition(),
            wpimath.kinematics.SwerveModulePosition(),
            wpimath.kinematics.SwerveModulePosition(),
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading(),
            self.swerve_positions,
        )

        self.front_right = subsystems.swerve_module.SwerveModule(
            can_ids.drive_front_right,
            can_ids.turn_front_right,
            can_ids.encoders_front_right,
        )

        self.front_left = subsystems.swerve_module.SwerveModule(
            can_ids.drive_front_left,
            can_ids.turn_front_left,
            can_ids.encoders_front_left,
        )

        self.back_right = subsystems.swerve_module.SwerveModule(
            can_ids.drive_back_right,
            can_ids.turn_back_right,
            can_ids.encoders_back_right,
        )

        self.back_left = subsystems.swerve_module.SwerveModule(
            can_ids.drive_back_left,
            can_ids.turn_back_left,
            can_ids.encoders_back_left,
        )

        self.swerve_modules = (
            self.front_right,
            self.front_left,
            self.back_right,
            self.back_left,
        )

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.ntcore_instance.getTable("Drivetrain")
        self.heading_topic = self.nt_table.getDoubleTopic("Heading")
        self.heading_publish = self.heading_topic.publish()
        self.heading_publish.set(0.0)

    def get_heading(self) -> wpimath.geometry.Rotation2d:
        return self.pigeon.getRotation2d()

    def get_pose(self) -> wpimath.geometry.Pose2d:
        return self.odometry.getPose()

    def add_vision_measurement(self, pose: wpimath.geometry.Pose2d) -> None:
        # TODO update local odometry with vision pose smoothly rather than instant jump
        self.reset_pose(pose)

    def reset_pose(self, pose: wpimath.geometry.Pose2d) -> None:
        self.odometry.resetPose(pose)

    def reset_gyro(self) -> None:
        self.pigeon.reset()

    def reset_gyro_cmd(self) -> ResetGyroCommand:
        return ResetGyroCommand(self)
    
    def telemetry(self) -> None:
        self.heading_publish.set(self.get_heading().degrees())

    def drive(self, forward: float, sideways: float, rotation: float) -> None:
        speeds = wpimath.kinematics.ChassisSpeeds(forward, sideways, rotation)
        self.swerve_states = self.kinematics.toSwerveModuleStates(speeds)

        self.swerve_modules[0].set_state(self.swerve_states[0])
        self.swerve_modules[1].set_state(self.swerve_states[1])
        self.swerve_modules[2].set_state(self.swerve_states[2])
        self.swerve_modules[3].set_state(self.swerve_states[3])

    def drive_cmd(self, forward: float, sideways: float, rotation: float) -> DriveRobotCommand:
        return DriveRobotCommand(self, forward, sideways, rotation)

    def periodic(self) -> None:
        self.odometry.update(
            self.get_heading(),
            self.swerve_positions,
        )

class ResetGyroCommand(commands2.command.Command):
    def __init__(self, drivetrain: SwerveDrivetrain):
        self.drivetrain = drivetrain

    def execute(self):
        self.drivetrain.reset_gyro()

class DriveRobotCommand(commands2.command.Command):
    def __init__(self, drivetrain: SwerveDrivetrain, forward: float, sideways: float, rotation: float):
        self.drivetrain = drivetrain
        self.forward = forward
        self.sideways = sideways
        self.rotation = rotation

    def execute(self):
        # TODO
        pass
