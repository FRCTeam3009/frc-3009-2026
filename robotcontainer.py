#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
import wpilib
import ntcore
import wpimath.geometry
import automodes
import subsystems.limelight
import subsystems.shooter
import subsystems.intake
import subsystems.climber
import subsystems.drive_robot_relative
import subsystems.swerve_drivetrain
import swerve_params


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.driver_controller = commands2.button.CommandXboxController(0)
        self.operator_controller = commands2.button.CommandXboxController(1)

        self.drivetrain = subsystems.swerve_drivetrain.SwerveDrivetrain()
        commands2.CommandScheduler.getInstance().registerSubsystem(self.drivetrain)

        self.drivetrain.reset_pose(wpimath.geometry.Pose2d(10, 2, 0))

        self.front_limelight = subsystems.limelight.Limelight("limelight-front", self.drivetrain)

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

        def forward_drive() -> float:
            output = -1 * self.driver_controller.getLeftY() * swerve_params.speed * self.speed_limit
            if abs(output) < 0.01:
                return 0
            return output
        def sideways_drive() -> float:
            output = -1 * self.driver_controller.getLeftX() * swerve_params.speed * self.speed_limit
            if abs(output) < 0.01:
                return 0
            return output
        def rotation_drive() -> float:
            output = -1 * self.driver_controller.getRightX() * swerve_params.angular_rate * self.speed_limit
            if abs(output) < 0.01:
                return 0
            return output
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.drive_cmd(
                forward_drive,
                sideways_drive,
                rotation_drive,
            )
        )

        # # Drive Robot-relative for small adjustments with D-pad
        # self.driver_controller.povUp().whileTrue(
        #     subsystems.drive_robot_relative.drive_forward_command(
        #         self.drivetrain,
        #         1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #     )
        # )
        # self.driver_controller.povDown().whileTrue(
        #     subsystems.drive_robot_relative.drive_forward_command(
        #         self.drivetrain,
        #         -1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #     )
        # )
        # self.driver_controller.povRight().whileTrue(
        #     subsystems.drive_robot_relative.drive_sideways_command(
        #         self.drivetrain,
        #         1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #     )
        # )
        # self.driver_controller.povLeft().whileTrue(
        #     subsystems.drive_robot_relative.drive_sideways_command(
        #         self.drivetrain,
        #         1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #     )
        # )
        # self.driver_controller.povUpRight().whileTrue(
        #     subsystems.drive_robot_relative.drive_command(
        #         self.drivetrain,
        #         1.0,
        #         1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #         0.0,
        #     )
        # )
        # self.driver_controller.povDownRight().whileTrue(
        #     subsystems.drive_robot_relative.drive_command(
        #         self.drivetrain,
        #         -1.0,
        #         1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #         0.0,
        #     )
        # )
        # self.driver_controller.povDownLeft().whileTrue(
        #     subsystems.drive_robot_relative.drive_command(
        #         self.drivetrain,
        #         -1.0,
        #         -1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #         0.0,
        #     )
        # )
        # self.driver_controller.povUpLeft().whileTrue(
        #     subsystems.drive_robot_relative.drive_command(
        #         self.drivetrain,
        #         1.0,
        #         -1.0,
        #         subsystems.drive_robot_relative.NORMAL_SPEED,
        #         0.0,
        #     )
        # )

        # reset the field-centric heading on left bumper press
        reset_gyro = self.drivetrain.reset_gyro_cmd()
        self.driver_controller.back().onTrue(reset_gyro)
        
        def shoot_speed():
            return -1 * self.operator_controller.getRightTriggerAxis()
        self.operator_controller.rightTrigger(0.1).whileTrue(
            self.shooter.fire_cmd(shoot_speed)
        )
        
        def shoot_backwards():
            return self.shooter.backwards_speed
        self.operator_controller.leftTrigger(0.1).whileTrue(
            self.shooter.fire_cmd(shoot_backwards)
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
        self.operator_controller.a().onTrue(
            self.climber.UpperLatchCmd()
        )
        self.operator_controller.x().onTrue(
            self.climber.LowerLatchCmd()
        )
        (self.driver_controller.back() & self.driver_controller.start()).whileTrue(
            self.front_limelight.reset_pose_command(self.drivetrain)
        )
        
    
    def get_auto_command(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """

        auto_mode = self.auto_dashboard.get_current_auto_builder(self.drivetrain, self.shooter, self.climber)
        return auto_mode
    
    def telemetry(self):
        self.periodic_publish.set(self.periodic_timer.get())
        self.periodic_timer.reset()
        self.front_limelight.telemetry()
        self.drivetrain.telemetry()

        