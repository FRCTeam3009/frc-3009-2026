import commands2
import rev
import wpimath
import wpimath.system.plant
import typing
import ntcore
import can_ids

class Shooter(commands2.Subsystem):
    asdf = 5

    def __init__(self):
        self.motor = rev.SparkMax(can_ids.shooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))

        self.ramp_motor = rev.SparkMax(can_ids.ramp, rev.SparkLowLevel.MotorType.kBrushless)
        self.ramp_motor_sim = rev.SparkSim(self.ramp_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.backwards_speed = 0.25

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.shooter_table = self.ntcore_instance.getTable("Shooter")

        # Multiplier for the speed of the shooter motor. (e.g. 0.75)
        self.shooter_speed = 0.75
        self.shooter_topic = self.shooter_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.shooter_topic.publish()
        self.motor_speed_publish.set(self.shooter_speed)
        self.motor_speed_subscribe = self.shooter_topic.subscribe(self.shooter_speed)

        # Speed in RPMs that we want the shooter motor to reach before we start firing. (e.g. 5000)
        self.ramp_up_speed = 6700
        self.ramp_up_speed_topic = self.shooter_table.getFloatTopic("ramp_up_speed")
        self.ramp_up_speed_publish = self.ramp_up_speed_topic.publish()
        self.ramp_up_speed_publish.set(self.ramp_up_speed)
        self.ramp_up_speed_subscribe = self.ramp_up_speed_topic.subscribe(self.ramp_up_speed)

        # Multiplier for the speed of the motor that pulls balls out of the hopper into the shooter. (e.g. 0.25)
        self.ramp_motor_speed = 0.25
        self.ramp_motor_speed_topic = self.shooter_table.getFloatTopic("ramp_motor_speed")
        self.ramp_motor_speed_publish = self.ramp_motor_speed_topic.publish()
        self.ramp_motor_speed_publish.set(self.ramp_motor_speed)
        self.ramp_motor_speed_subscribe = self.ramp_motor_speed_topic.subscribe(self.ramp_motor_speed)
    
    def move(self, speed: float):
        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)

    def fire_cmd(self, speed: typing.Callable[[], float]):
        return FireCommand(self, speed)

class FireCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: typing.Callable[[], float]):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.speed = speed

    def execute(self):
        # Start running the shooter motor
        shooter_speed = self.shooter.motor_speed_subscribe.get()
        self.shooter.move(self.speed() * shooter_speed)

        # Wait until the shooter motor is up to speed before loading balls into it.
        current_speed = self.shooter.motor.getEncoder().getVelocity()
        ramp_up_speed = self.shooter.ramp_up_speed_subscribe.get()
        ramp_motor_speed = self.shooter.ramp_motor_speed_subscribe.get()
        if current_speed > ramp_up_speed:
            self.shooter.ramp_motor.set(ramp_motor_speed)

    def end(self, interrupted: bool):
        self.shooter.move(0)
        self.shooter.ramp_motor.set(0)