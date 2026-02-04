import commands2
import rev
import wpimath
import wpimath.system.plant
import typing
import ntcore
import can_ids

class Shooter(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.motor = rev.SparkMax(can_ids.shooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))

        shooter_speed = 1

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.shooter_table = self.ntcore_instance.getTable("Shooter")
        self.shooter_topic = self.shooter_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.shooter_topic.publish()
        self.motor_speed_publish.set(shooter_speed)
        self.motor_speed_subscribe = self.shooter_topic.subscribe(shooter_speed)
    
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
        shooter_speed = self.shooter.motor_speed_subscribe.get()
        self.shooter.move(self.speed() * shooter_speed)

    def end(self, interrupted):
        self.shooter.move(0)

    #TODO very basic auto shoots drives (add in drive the auto only shoots right now)
    #TODO add climber