import commands2
import rev
import wpimath
import wpimath.system.plant
import typing
import ntcore
import can_ids
import subsystems.intake

class Shooter(commands2.Subsystem):

    def __init__(self, intake: subsystems.intake.Intake):
        self.motor = rev.SparkMax(can_ids.shooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))

        self.ramp_motor = rev.SparkMax(can_ids.ramp, rev.SparkLowLevel.MotorType.kBrushless)
        self.ramp_motor_sim = rev.SparkSim(self.ramp_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.backwards_speed = 1500

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.shooter_table = self.ntcore_instance.getTable("Shooter")

        # RPMs for the speed of the shooter motor. (e.g. 3000)
        self.max_speed = 6784 # Spark Neo Vortex free speed RPMs
        self.shooter_speed = 3000
        self.shooter_topic = self.shooter_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.shooter_topic.publish()
        self.motor_speed_publish.set(self.shooter_speed)
        self.motor_speed_subscribe = self.shooter_topic.subscribe(self.shooter_speed)

        self.big_shot_speed = 5000
        self.big_shot_topic = self.shooter_table.getFloatTopic("BigShotSpeed")
        self.big_shot_publish = self.big_shot_topic.publish()
        self.big_shot_publish.set(self.big_shot_speed)
        self.big_shot_subscribe = self.big_shot_topic.subscribe(self.big_shot_speed)

        # Multiplier for the speed of the motor that pulls balls out of the hopper into the shooter. (e.g. 0.25)
        self.ramp_motor_speed = 0.75
        self.automatic_ramp_speed = -0.25
        self.ramp_motor_speed_topic = self.shooter_table.getFloatTopic("ramp_motor_speed")
        self.ramp_motor_speed_publish = self.ramp_motor_speed_topic.publish()
        self.ramp_motor_speed_publish.set(self.ramp_motor_speed)
        self.ramp_motor_speed_subscribe = self.ramp_motor_speed_topic.subscribe(self.ramp_motor_speed)

        self.intake = intake
    
    def move(self, speed: float):
        # Speed is in RPMs, so convert to 1.0 range
        speed = speed / self.max_speed

        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)

    def fire_cmd(self, speed: float):
        return FireCommand(self, speed, self.intake)
    
    def idle_cmd(self):
        return IdleCommand(self, self.intake)
    
    def at_speed(self, speed: float) -> bool:
        v = self.motor.getEncoder().getVelocity()

        # Check if we're within some boundary slack of RPMs
        return abs(v - speed) < 200

class FireCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: float, intake: subsystems.intake.Intake):
        self.addRequirements(shooter)
        self.addRequirements(intake)
        self.intake = intake
        self.shooter = shooter
        self.speed = speed

    def execute(self):
        # Start running the rollers to pull balls into the shooter
        self.intake.is_running = True
        self.intake.RunRollers()

        # Wait until the shooter motor is up to speed before loading balls into it.
        ramp_motor_speed = self.shooter.ramp_motor_speed_subscribe.get()

         # Start running the shooter motor
        self.shooter.move(self.speed)

        backwards = self.speed > 0 # Motor is inverted

        if self.shooter.at_speed(self.speed):
            self.shooter.ramp_motor.set(ramp_motor_speed)
        elif backwards:
            self.shooter.ramp_motor.set(-1 * ramp_motor_speed)
            self.intake.RunRollersBackwards()

    def end(self, interrupted: bool):
        self.shooter.move(0)
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
