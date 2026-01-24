import commands2
import rev
import wpimath
import wpimath.system.plant
import typing

class Shooter(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.motor = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))
    
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
    





    #TODO shooter command
    #TODO very basic auto shoots drives
    #TODO Network table dashboard configure values dynamically (shooter speed)
    #TODO add climber