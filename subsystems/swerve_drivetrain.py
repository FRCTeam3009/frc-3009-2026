import wpimath.geometry
import commands2.command
import ntcore
import phoenix6
import can_ids
import wpimath.kinematics
import wpimath.units
import subsystems.swerve_module
import typing
import simulation
import swerve_params

class SwerveDrivetrain(commands2.Subsystem):
    def __init__(self):
        self.pigeon = phoenix6.hardware.Pigeon2(can_ids.pigeon)

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.ntcore_instance.getTable("Drivetrain")

        # 21 inches horizontal
        # 21.25 inches forward
        horizontal = 10.5
        forward = 10.625
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            # front left
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(forward), wpimath.units.inchesToMeters(horizontal)),
            # front right
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(forward), wpimath.units.inchesToMeters(-horizontal)),
            # back left
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(-forward), wpimath.units.inchesToMeters(horizontal)),
            # back right
            wpimath.geometry.Translation2d(wpimath.units.inchesToMeters(-forward), wpimath.units.inchesToMeters(-horizontal)),
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
            "FrontRight",
            can_ids.drive_front_right,
            can_ids.turn_front_right,
            can_ids.encoders_front_right,
            swerve_params.encoder_offset_front_right,
            self.nt_table,
        )

        self.front_left = subsystems.swerve_module.SwerveModule(
            "FrontLeft",
            can_ids.drive_front_left,
            can_ids.turn_front_left,
            can_ids.encoders_front_left,
            swerve_params.encoder_offset_front_left,
            self.nt_table,
        )

        self.back_right = subsystems.swerve_module.SwerveModule(
            "BackRight",
            can_ids.drive_back_right,
            can_ids.turn_back_right,
            can_ids.encoders_back_right,
            swerve_params.encoder_offset_back_right,
            self.nt_table,
        )

        self.back_left = subsystems.swerve_module.SwerveModule(
            "BackLeft",
            can_ids.drive_back_left,
            can_ids.turn_back_left,
            can_ids.encoders_back_left,
            swerve_params.encoder_offset_back_left,
            self.nt_table,
        )

        self.swerve_modules = (
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
        )

        self.heading_topic = self.nt_table.getDoubleTopic("Heading")
        self.heading_publish = self.heading_topic.publish()
        self.heading_publish.set(0.0)

        self.position_topic = self.nt_table.getStructTopic("Position", wpimath.geometry.Pose2d)
        self.position_publish = self.position_topic.publish()
        self.position_publish.set(wpimath.geometry.Pose2d())

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
        self.position_publish.set(self.get_pose())

        for module in self.swerve_modules:
            module.telemetry()

    def drive(self, forward: float, sideways: float, rotation: float) -> None:
        speeds = wpimath.kinematics.ChassisSpeeds(forward, sideways, rotation)
        self.swerve_states = self.kinematics.toSwerveModuleStates(speeds)

        if simulation.is_simulation and abs(rotation) > 0:
            current = self.get_heading().degrees()
            self.pigeon.set_yaw(current + wpimath.units.radiansToDegrees(rotation))

        i = 0
        while i < 4:
            module = self.swerve_modules[i]
            state = self.swerve_states[i]
            module.set_state(state)
            i += 1

    def drive_cmd(
            self, 
            forward: typing.Callable[[], float], 
            sideways: typing.Callable[[], float], 
            rotation: typing.Callable[[], float],
            ) -> DriveRobotCommand:
        return DriveRobotCommand(self, forward, sideways, rotation)
    
    def update_positions(self) -> typing.Tuple[
        wpimath.kinematics.SwerveModulePosition,
        wpimath.kinematics.SwerveModulePosition,
        wpimath.kinematics.SwerveModulePosition,
        wpimath.kinematics.SwerveModulePosition,
        ]:
        return (
            wpimath.kinematics.SwerveModulePosition(
                self.swerve_modules[0].get_distance(), self.swerve_modules[0].get_angle()),
            wpimath.kinematics.SwerveModulePosition(
                self.swerve_modules[1].get_distance(), self.swerve_modules[1].get_angle()),
            wpimath.kinematics.SwerveModulePosition(
                self.swerve_modules[2].get_distance(), self.swerve_modules[2].get_angle()),
            wpimath.kinematics.SwerveModulePosition(
                self.swerve_modules[3].get_distance(), self.swerve_modules[3].get_angle()),
        )

    def periodic(self) -> None:
        self.swerve_positions = self.update_positions()
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
    def __init__(
            self, 
            drivetrain: SwerveDrivetrain, 
            forward: typing.Callable[[], float], 
            sideways: typing.Callable[[], float], 
            rotation: typing.Callable[[], float],
            ):
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain
        self.forward = forward
        self.sideways = sideways
        self.rotation = rotation

    def execute(self) -> None:
        self.drivetrain.drive(self.forward(), self.sideways(), self.rotation())
    
    def isFinished(self) -> bool:
        return False
