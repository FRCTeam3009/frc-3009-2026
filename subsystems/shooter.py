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

    def __init__(self):
        self.motor = rev.SparkMax(can_ids.shooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))
        self.secondarymotor = rev.SparkMax(can_ids.secondary_shooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.secondarymotor_sim = rev.SparkSim(self.secondarymotor, wpimath.system.plant.DCMotor.NEO(1))
        self.max_speed = 6784 # Spark Neo Vortex free speed RPMs

        self.motor_bang_bang_primary = wpimath.controller.BangBangController()
        self.motor_bang_bang_secondary = wpimath.controller.BangBangController()

        self.ramp_motor = rev.SparkMax(can_ids.ramp, rev.SparkLowLevel.MotorType.kBrushless)
        self.ramp_motor_sim = rev.SparkSim(self.ramp_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.backwards_speed = 0.25 # 1500

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.shooter_table = self.ntcore_instance.getTable("Shooter")

        # RPMs for the speed of the shooter motor. (e.g. 3000)
        self.shooter_speed = 3000
        self.motor_speed_topic = self.shooter_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.motor_speed_topic.publish()
        self.motor_speed_publish.set(self.shooter_speed)
        self.motor_speed_subscribe = self.motor_speed_topic.subscribe(self.shooter_speed)

        self.current_motor_speed_topic = self.shooter_table.getFloatTopic("CurrentMotorSpeed")
        self.current_motor_speed_publish = self.current_motor_speed_topic.publish()
        self.current_motor_speed_publish.set(0.0)

        self.secondary_current_motor_speed_topic = self.shooter_table.getFloatTopic("CurrentSecondaryMotorSpeed")
        self.secondary_current_motor_speed_publish = self.secondary_current_motor_speed_topic.publish()
        self.secondary_current_motor_speed_publish.set(0.0)

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
    
    # set_speed simply sets the motor speed directly.
    def set_speed_primary(self, speed: float):
        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)

    def set_speed_secondary(self, speed: float):
        self.secondarymotor.set(speed)
        self.secondarymotor_sim.setAppliedOutput(speed)
        self.secondarymotor_sim.setPosition(self.secondarymotor_sim.getPosition() + speed * 2)
        self.secondarymotor_sim.getAbsoluteEncoderSim().setPosition(self.secondarymotor_sim.getPosition() + speed * 2)

    def get_shooter_speed(self):
        return self.motor_speed_subscribe.get()
    
    def get_big_shot_speed(self):
        return self.big_shot_subscribe.get()

    # set_flywheel tries to maintain speed using a BangBangController
    def set_flywheel_primary(self, speed: float):
        current_speed = self.motor.getEncoder().getVelocity()
        bangbang = self.motor_bang_bang_primary.calculate(current_speed, speed)
        self.set_speed_primary(bangbang*0.8)

    def set_flywheel_secondary(self, speed: float):
        current_speed = self.secondarymotor.getEncoder().getVelocity()
        bangbang = self.motor_bang_bang_secondary.calculate(current_speed, speed)
        self.set_speed_secondary(bangbang*0.8)

    def fire_cmd(self, speed: typing.Callable[[], float]):
        return FireCommand(self, speed)
    
    def backwards_cmd(self, intake: subsystems.intake.Intake, speed: float):
        return BackwardsCommand(self, speed, intake)
    
    def idle_cmd(self, intake: subsystems.intake.Intake):
        return IdleCommand(self, intake)
    
    def up_to_speed(self, speed: float) -> bool:
        velocity1 = self.motor.getEncoder().getVelocity()
        velocity2 = self.secondarymotor.getEncoder().getVelocity()

        return velocity1 > speed and velocity2 > speed
    
    def telemetry(self):
        self.current_motor_speed_publish.set(self.motor.getEncoder().getVelocity())
        self.secondary_current_motor_speed_publish.set(self.secondarymotor.getEncoder().getVelocity())

class FireCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: typing.Callable[[], float]):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.speed = speed

    def execute(self):
         # Start running the shooter motor
        self.shooter.set_flywheel_primary(self.speed())
        self.shooter.set_flywheel_secondary(self.speed())

        # Wait until the shooter motor is up to speed before loading balls into it.
        ramp_motor_speed = self.shooter.ramp_motor_speed_subscribe.get()
        if self.shooter.up_to_speed(self.speed()):
            self.shooter.ramp_motor.set(ramp_motor_speed)
        else:
            self.shooter.ramp_motor.set(0)

    def end(self, interrupted: bool):
        self.shooter.set_speed_primary(0)
        self.shooter.set_speed_secondary(0)
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
        self.intake.RunIntakeBackwards()
        self.shooter.set_speed_primary(-1 * self.speed)
        self.shooter.set_speed_secondary(-1 * self.speed)

    def end(self, interrupted: bool):
        self.shooter.set_speed_primary(0)
        self.shooter.set_speed_secondary(0)
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
