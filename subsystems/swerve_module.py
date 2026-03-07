import phoenix6.hardware
import rev
import wpimath.kinematics
import wpimath.geometry
import phoenix6.swerve.utility.phoenix_pid_controller
import swerve_params
import phoenix6.configs
import phoenix6.controls

class SwerveModule():
    def __init__(self, drive: int, turn: int, encoder: int):
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
        self.drive.configurator.apply(self.drive_pid_config)

        self.drive_velocity = phoenix6.controls.VelocityVoltage(0)

    def set_state(self, state: wpimath.kinematics.SwerveModuleState):
        self.turn_pid.setSetpoint(state.angle.degrees(), rev.SparkLowLevel.ControlType.kPosition)

        rotations_per_second = state.speed / swerve_params.wheel_circumference
        self.drive.set_control(self.drive_velocity.with_velocity(rotations_per_second))

    def get_angle(self) -> wpimath.geometry.Rotation2d:
        rotation = self.encoder.get_position()
        return wpimath.geometry.Rotation2d.fromRotations(rotation.value_as_double)
    
    def get_distance(self) -> float:
        return self.drive.get_position().value_as_double
