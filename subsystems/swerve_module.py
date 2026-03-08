import phoenix6.hardware
import rev
import wpimath.kinematics
import wpimath.geometry
import phoenix6.swerve.utility.phoenix_pid_controller
import swerve_params
import phoenix6.configs
import phoenix6.controls
import ntcore
import simulation
import wpimath.controller
import math

class SwerveModule():
    def __init__(self, name: str, drive: int, turn: int, encoder: int, encoder_offset: float, table: ntcore.NetworkTable):
        self.drive = phoenix6.hardware.TalonFX(drive)
        self.turn = rev.SparkMax(turn, rev.SparkLowLevel.MotorType.kBrushless)
        self.encoder = phoenix6.hardware.CANcoder(encoder)
        encoder_config = phoenix6.configs.CANcoderConfiguration()
        if simulation.is_simulation:
            encoder_offset = 0.0
        encoder_config.magnet_sensor.magnet_offset = -1 * encoder_offset
        self.encoder.configurator.apply(encoder_config)

        self.turn_pid = wpimath.controller.PIDController(
            swerve_params.turn_p,
            swerve_params.turn_i,
            swerve_params.turn_d,
        )
        self.turn_pid.enableContinuousInput(0, 90)

        self.drive_pid = phoenix6.swerve.utility.phoenix_pid_controller.PhoenixPIDController(
            swerve_params.drive_p,
            swerve_params.drive_i,
            swerve_params.drive_d,
        )

        self.drive_pid_config = phoenix6.configs.Slot0Configs()
        self.drive_pid_config.k_p = swerve_params.drive_p
        self.drive_pid_config.k_i = swerve_params.drive_i
        self.drive_pid_config.k_d = swerve_params.drive_d
        self.drive_pid_config.k_s = swerve_params.drive_s
        self.drive_pid_config.k_v = swerve_params.drive_v
        self.drive.configurator.apply(self.drive_pid_config)

        self.drive_velocity = phoenix6.controls.VelocityVoltage(0)

        self.nt_table = table.getSubTable(name)

        self.rotation_topic = self.nt_table.getDoubleTopic("Rotation")
        self.rotation_topic_publish = self.rotation_topic.publish()
        self.rotation_topic_publish.set(0.0)

        self.drive_speed_topic = self.nt_table.getDoubleTopic("DriveSpeed")
        self.drive_speed_publish = self.drive_speed_topic.publish()
        self.drive_speed_publish.set(0.0)

        self.turn_speed_topic = self.nt_table.getDoubleTopic("TurnSpeed")
        self.turn_speed_publish = self.turn_speed_topic.publish()
        self.turn_speed_publish.set(0.0)

        self.turn_setpoint_topic = self.nt_table.getDoubleTopic("TurnSetpoint")
        self.turn_setpoint_publish = self.turn_setpoint_topic.publish()
        self.turn_setpoint_publish.set(0.0)
        self.turn_setpoint = 0.0

        self.drive_setpoint_topic = self.nt_table.getDoubleTopic("DriveSetpoint")
        self.drive_setpoint_publish = self.drive_setpoint_topic.publish()
        self.drive_setpoint_publish.set(0.0)
        self.drive_setpoint = 0.0

        self.encoder_topic = self.nt_table.getDoubleTopic("Encoder")
        self.encoder_publish = self.encoder_topic.publish()
        self.encoder_publish.set(0.0)

    def telemetry(self):
        self.rotation_topic_publish.set(self.get_angle().degrees())
        self.drive_speed_publish.set(self.drive.get_velocity().value_as_double)
        self.turn_speed_publish.set(self.turn.get())
        self.turn_setpoint_publish.set(self.turn_setpoint)
        self.drive_setpoint_publish.set(self.drive_setpoint)
        self.encoder_publish.set(self.get_angle().degrees() % 360)

    def set_state(self, state: wpimath.kinematics.SwerveModuleState):
        self.turn_setpoint = state.angle.degrees()
        turn_pos = self.get_angle().radians()
        turn_value = self.turn_pid.calculate(turn_pos, state.angle.radians())
        if turn_value > 0.01:
            self.turn.set(turn_value)

        rotations_per_second = state.speed / swerve_params.wheel_circumference
        self.drive_setpoint = rotations_per_second
        self.drive.set_control(self.drive_velocity.with_velocity(rotations_per_second))

        if simulation.is_simulation:
            current_drive = self.drive.get_position().value_as_double
            rotations_drive = state.speed / 100
            self.drive.set_position(current_drive + rotations_drive)

            current_turn = self.turn.getEncoder().getPosition()
            rotations_turn = state.angle.degrees() / 360
            self.turn.getEncoder().setPosition(current_turn + rotations_turn)

    def get_angle(self) -> wpimath.geometry.Rotation2d:
        rotation = self.encoder.get_position()
        return wpimath.geometry.Rotation2d.fromRotations(rotation.value_as_double)
    
    def get_distance(self) -> float:
        return self.drive.get_position().value_as_double
