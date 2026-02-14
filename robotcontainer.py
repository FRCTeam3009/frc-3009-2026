#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
import subsystems.drive_robot_relative
import telemetry
import wpilib
import ntcore
import wpimath.geometry

from phoenix6 import swerve
from wpimath.units import rotationsToRadians
import subsystems.limelight
import automodes
import subsystems.shooter
import subsystems.intake
import subsystems.climber


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.001)
            .with_rotational_deadband(
                self._max_angular_rate * 0.001
            )  # Add a 0.1% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        self.logger = telemetry.Telemetry(self._max_speed)

        self.driver_controller = commands2.button.CommandXboxController(0)
        self.operator_controller = commands2.button.CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.drivetrain.reset_pose(wpimath.geometry.Pose2d(10, 2, 0))

        self.front_limelight = subsystems.limelight.Limelight("limelight-front", self.drivetrain)
        self.back_limelight = subsystems.limelight.Limelight("limelight-back", self.drivetrain)

        self.auto_dashboard = automodes.AutoDashboard()

        self.periodic_timer = wpilib.Timer()
        self.periodic_timer.start()

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.container_table = self.ntcore_instance.getTable("robotcontainer")
        self.periodic_topic = self.container_table.getFloatTopic("periodic_timer")
        self.periodic_publish = self.periodic_topic.publish()
        self.periodic_publish.set(0.0)

        self.speed_limit = subsystems.drive_robot_relative.NORMAL_SPEED

        self.shooter = subsystems.shooter.Shooter()
        commands2.CommandScheduler.getInstance().registerSubsystem(self.shooter)

        self.intake = subsystems.intake.Intake()
        commands2.CommandScheduler.getInstance().registerSubsystem(self.intake)

        self.climber = subsystems.climber.Climber()
        commands2.CommandScheduler.getInstance().registerSubsystem(self.climber)

        # Configure the button bindings
        self.configure_button_bindings()
        
        self.front_limelight.update_command().schedule()
        self.back_limelight.update_command().schedule()
        #self.front_limelight.odometry_command().schedule()

    def set_turbo_speed(self):
        self.speed_limit = subsystems.drive_robot_relative.TURBO_SPEED

    def set_normal_speed(self):
        self.speed_limit = subsystems.drive_robot_relative.NORMAL_SPEED

    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self.driver_controller.rightTrigger().whileTrue(
            commands2.cmd.run(self.set_turbo_speed).finallyDo(lambda interrupted: self.set_normal_speed())
        )

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        #
        #         X
        #         ^
        #         |
        #    Y <--*
        #
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self.driver_controller.getLeftY() * self._max_speed * self.speed_limit
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self.driver_controller.getLeftX() * self._max_speed * self.speed_limit
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self.driver_controller.getRightX() * self._max_angular_rate * self.speed_limit
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self.driver_controller.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(0)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povDown().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(0)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(0.0)
                .with_velocity_y(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(0.0)
                .with_velocity_y(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povUpRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povDownRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povDownLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )
        self.driver_controller.povUpLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate / 2)
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self.driver_controller.back() & self.driver_controller.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.driver_controller.back() & self.driver_controller.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.driver_controller.start() & self.driver_controller.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.driver_controller.start() & self.driver_controller.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        def field_centric():
            return self.drivetrain.seed_field_centric()
        self.driver_controller.leftBumper().onTrue(
            self.drivetrain.runOnce(field_centric)
        )

        def telemetry_func(state):
            return self.logger.telemeterize(state)
        self.drivetrain.register_telemetry(telemetry_func)

        self.driver_controller.b().whileTrue(
            subsystems.drive_robot_relative.drive_command(self.drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, self.speed_limit, 0)
        )
        
        def shoot_speed():
            return -1 * self.operator_controller.getRightTriggerAxis()
        self.operator_controller.rightTrigger().whileTrue(
            self.shooter.fire_cmd(shoot_speed)
        )

        self.operator_controller.povUp().whileTrue(
            self.climber.move_cmd(self.climber.climber_speed)
        )
        self.operator_controller.povDown().whileTrue(
            self.climber.move_cmd(-self.climber.climber_speed)
        )
        self.operator_controller.b().onTrue(
            self.intake.InNOutCmd()
        )
        self.operator_controller.y().whileTrue(
            self.intake.IntakeCmd()
        )
        (self.driver_controller.back() & self.driver_controller.start()).whileTrue(
            self.front_limelight.reset_pose_command(self.drivetrain)
        )

        # TODO use this to know which side is active.
        # This data is empty before the start.
        # It is only updated once at the end of auto.
        # It is either 'R' or 'B' for red and blue.
        # It specifies which side will go *inactive* first (both are active at the start)
        # TODO Create a new class that polls for this until it gets a value.
        # Then it can start timers to determine which field is currently active.
        # We can publish the timer data too for countdown info.
        # Test it with the full practice mode countdown.
        #
        # wpilib.DriverStation.getGameSpecificMessage()

    
    def get_auto_command(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """

        auto_mode = self.auto_dashboard.get_current_auto_builder(self.drivetrain, self.shooter)
        return auto_mode
    
    def telemetry(self):
        self.periodic_publish.set(self.periodic_timer.get())
        self.periodic_timer.reset()
        self.front_limelight.telemetry()

        