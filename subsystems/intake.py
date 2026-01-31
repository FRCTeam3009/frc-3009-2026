import wpilib
import commands2

class Intake(commands2.Subsystem):
    def __init__(self):
        self.HorizontalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 0, 1)
        self.VerticalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)
        self.HorizontalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.VerticalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)

    def Vertical(self):
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