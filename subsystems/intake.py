import wpilib
import commands2

class Intake(commands2.Subsystem):
    def __init__(self):
        self.HorizontalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 0, 1)
        self.VerticalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)
        self.HorizontalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.VerticalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)

        # TODO add motor for actually picking up balls.

    def Vertical(self):
        # TODO prevent us from going down unless we are already out.
        # TODO prevent us from going up unless we are also already out.
        # TODO basically, don't let us move vertically unless the horizontal position is out and deployed first.
        # Might be better to simplify this into a "DeployCommand" or something that goes out and then down.
        self.VerticalMotion.toggle()

    def Horizontal(self):
        self.HorizontalMotion.toggle()

    def HorizontalCmd(self):
        return IntakeHorizontalCommand(self)
    
    def VerticalCmd(self):
        return IntakeVerticalCommand(self)

class IntakeHorizontalCommand(commands2.Command):
    def __init__(self, intake: Intake):
        self.addRequirements(intake)
        self.intake = intake

    def execute(self):
        self.intake.Horizontal()

    def isFinished(self):
        return True

class IntakeVerticalCommand(commands2.Command):
    def __init__(self, intake: Intake):
        self.addRequirements(intake)
        self.intake = intake

    def execute(self):
        self.intake.Vertical()

    def isFinished(self):
        return True
    
# TODO add command to turn on the motor to pick up balls off the ground.