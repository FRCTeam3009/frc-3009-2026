import commands2
import rev
import wpimath
import wpimath.system.plant
import typing
import ntcore
import can_ids
import subsystems.intake
import wpimath.controller

class Shooter(commands2.Subsystem):

    def __init__(self, intake: subsystems.intake.Intake):
        self.motor = rev.SparkMax(can_ids.shooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))
        self.max_speed = 6784 # Spark Neo Vortex free speed RPMs
        self.motor_bang_bang = wpimath.controller.BangBangController()

        self.ramp_motor = rev.SparkMax(can_ids.ramp, rev.SparkLowLevel.MotorType.kBrushless)
        self.ramp_motor_sim = rev.SparkSim(self.ramp_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.backwards_speed = 0.25 # 1500

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.shooter_table = self.ntcore_instance.getTable("Shooter")

        self.kS = 0.016
        self.kV = 0
        self.motor_tune_topic = self.shooter_table.getFloatTopic("MotorTuning")
        self.motor_tune_publish = self.motor_tune_topic.publish()
        self.motor_tune_publish.set(self.kV)
        self.motor_tune_subscribe = self.motor_tune_topic.subscribe(self.kV)
        self.kA = 0
        self.kP = 0
        self.kI = 0
        self.kD = 0

        # RPMs for the speed of the shooter motor. (e.g. 3000)
        self.shooter_speed = 3000
        self.motor_speed_topic = self.shooter_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.motor_speed_topic.publish()
        self.motor_speed_publish.set(self.shooter_speed)
        self.motor_speed_subscribe = self.motor_speed_topic.subscribe(self.shooter_speed)

        self.current_motor_speed_topic = self.shooter_table.getFloatTopic("CurrentMotorSpeed")
        self.current_motor_speed_publish = self.current_motor_speed_topic.publish()
        self.current_motor_speed_publish.set(0.0)

        self.big_shot_speed = 5000
        self.big_shot_topic = self.shooter_table.getFloatTopic("BigShotSpeed")
        self.big_shot_publish = self.big_shot_topic.publish()
        self.big_shot_publish.set(self.big_shot_speed)
        self.big_shot_subscribe = self.big_shot_topic.subscribe(self.big_shot_speed)

        # Multiplier for the speed of the motor that pulls balls out of the hopper into the shooter. (e.g. 0.25)
        self.ramp_motor_speed = 0.85
        self.automatic_ramp_speed = -0.25
        self.ramp_motor_speed_topic = self.shooter_table.getFloatTopic("ramp_motor_speed")
        self.ramp_motor_speed_publish = self.ramp_motor_speed_topic.publish()
        self.ramp_motor_speed_publish.set(self.ramp_motor_speed)
        self.ramp_motor_speed_subscribe = self.ramp_motor_speed_topic.subscribe(self.ramp_motor_speed)

        self.intake = intake
    
    # set_speed simply sets the motor speed directly.
    def set_speed(self, speed: float):
        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)

    def get_shooter_speed(self):
        return self.motor_speed_subscribe.get()
    
    def get_big_shot_speed(self):
        return self.big_shot_subscribe.get()

    # set_flywheel tries to maintain speed using a BangBangController
    def set_flywheel(self, speed: float):
        current_speed = self.motor.getEncoder().getVelocity()
        val = self.motor_bang_bang.calculate(current_speed, speed)
        motor_feedforward = wpimath.controller.SimpleMotorFeedforwardRadians(self.kS, self.motor_tune_subscribe.get(), self.kA)
        motor_pid = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        # val = motor_pid.calculate(current_speed, speed) + motor_feedforward.calculate(current_speed)
        print(current_speed)
        self.set_speed(val)

    def fire_cmd(self, speed: typing.Callable[[], float]):
        return FireCommand(self, speed, self.intake)
    
    def backwards_cmd(self, speed: float):
        return BackwardsCommand(self, speed, self.intake)
    
    def idle_cmd(self):
        return IdleCommand(self, self.intake)
    
    def up_to_speed(self, speed: float) -> bool:
        velocity = self.motor.getEncoder().getVelocity()

        return velocity > speed
    
    def telemetry(self):
        self.current_motor_speed_publish.set(self.motor.getEncoder().getVelocity())

class FireCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: typing.Callable[[], float], intake: subsystems.intake.Intake):
        self.addRequirements(shooter)
        self.addRequirements(intake)
        self.intake = intake
        self.shooter = shooter
        self.speed = speed

    def execute(self):
        # Start running the rollers to pull balls into the shooter
        #self.intake.is_running = True
        #self.intake.RunRollers()

         # Start running the shooter motor
        self.shooter.set_flywheel(self.speed())

        # Wait until the shooter motor is up to speed before loading balls into it.
        ramp_motor_speed = self.shooter.ramp_motor_speed_subscribe.get()
        if self.shooter.up_to_speed(self.speed()):
            self.shooter.ramp_motor.set(ramp_motor_speed)
        else:
            self.shooter.ramp_motor.set(0)           

    def end(self, interrupted: bool):
        self.shooter.set_speed(0)
        self.shooter.ramp_motor.set(0)

class BackwardsCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: float, intake: subsystems.intake.Intake):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.intake = intake
        self.speed = speed

    def execute(self):
        ramp_motor_speed = self.shooter.ramp_motor_speed_subscribe.get()
        self.shooter.ramp_motor.set(-1 * ramp_motor_speed)
        self.intake.RunRollersBackwards()
        self.shooter.set_speed(-1 * self.speed)

    def end(self, interrupted: bool):
        self.shooter.set_speed(0)
        self.shooter.ramp_motor.set(0)

class IdleCommand(commands2.Command):
    def __init__(self, shooter: Shooter, intake: subsystems.intake.Intake):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.intake = intake

    def execute(self):
        if abs(self.intake.IntakeMotor.get()) > 0.01:
            self.shooter.ramp_motor.set(self.shooter.automatic_ramp_speed)
        else:
            self.shooter.ramp_motor.set(0)
