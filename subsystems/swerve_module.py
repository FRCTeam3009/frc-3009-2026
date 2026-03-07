import phoenix6.hardware
import rev
import wpimath.kinematics
import wpimath.geometry
import phoenix6.swerve.utility.phoenix_pid_controller
import swerve_params
import phoenix6.configs
import phoenix6.controls
import ntcore

class SwerveModule():
    def __init__(self, name: str, drive: int, turn: int, encoder: int, table: ntcore.NetworkTable):
        self.drive = phoenix6.hardware.TalonFX(drive)
        self.turn = rev.SparkMax(turn, rev.SparkLowLevel.MotorType.kBrushless)
        self.encoder = phoenix6.hardware.CANcoder(encoder)

        self.turn_pid = self.turn.getClosedLoopController()

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

    def telemetry(self):
        self.rotation_topic_publish.set(self.get_angle().degrees())
        self.drive_speed_publish.set(self.drive.get_velocity().value_as_double)
        self.turn_speed_publish.set(self.turn.get())
        self.turn_setpoint_publish.set(self.turn_setpoint)
        self.drive_setpoint_publish.set(self.drive_setpoint)

    def set_state(self, state: wpimath.kinematics.SwerveModuleState):
        self.turn_setpoint = state.angle.degrees()
        self.turn_pid.setSetpoint(state.angle.degrees(), rev.SparkLowLevel.ControlType.kPosition)

        rotations_per_second = state.speed / swerve_params.wheel_circumference
        self.drive_setpoint = rotations_per_second
        self.drive.set_control(self.drive_velocity.with_velocity(rotations_per_second))

    def get_angle(self) -> wpimath.geometry.Rotation2d:
        rotation = self.encoder.get_position()
        return wpimath.geometry.Rotation2d.fromRotations(rotation.value_as_double)
    
    def get_distance(self) -> float:
        return self.drive.get_position().value_as_double
