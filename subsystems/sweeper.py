import wpilib
import commands2

class Sweeper(commands2.Subsystem):
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
        return SweeperHorizontalCommand(self)
    
    def VerticalCmd(self):
        return SweeperVerticalCommand(self)

class SweeperHorizontalCommand(commands2.Command):
    def __init__(self, sweeper: Sweeper):
        self.addRequirements(sweeper)
        self.sweeper = sweeper

    def execute(self):
        self.sweeper.Horizontal()

    def isFinished(self):
        return True

class SweeperVerticalCommand(commands2.Command):
    def __init__(self, sweeper: Sweeper):
        self.addRequirements(sweeper)
        self.sweeper = sweeper

    def execute(self):
        self.sweeper.Vertical()

    def isFinished(self):
        return True