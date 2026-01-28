import commands2
import rev
import wpimath
import wpimath.system.plant
import typing
import ntcore

class Shooter(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.motor = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("Shooter")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)
    
    def move(self, speed: float):
        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)

class FireCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: typing.Callable[[], float]):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.speed = speed

    def execute(self):
        print("penguins")
        self.shooter.move(self.speed())

    def end(self, interrupted):
        self.shooter.move(0)
    





    #TODO shooter command (seems to work don't have network tables yet so can't test fully)
    #TODO very basic auto shoots drives (add in drive the auto only shoots right now)
    #TODO Network table dashboard configure values dynamically (shooter speed)
    #TODO add climber