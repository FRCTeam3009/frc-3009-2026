import wpilib
import commands2
import rev
import wpimath.system.plant
import can_ids

class Intake(commands2.Subsystem):
    def __init__(self):
        self.HorizontalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 0, 1)
        self.VerticalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)
        self.HorizontalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.VerticalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.IntakeMotor = rev.SparkMax(can_ids.intake, rev.SparkLowLevel.MotorType.kBrushless)
        self.IntakeMotorSim = rev.SparkSim(self.IntakeMotor, wpimath.system.plant.DCMotor.NEO(1))

        self.deploying = -1

    def VerticalState(self) -> wpilib.DoubleSolenoid.Value:
        return self.VerticalMotion.get()

    def HorizontalState(self) -> wpilib.DoubleSolenoid.Value:
        return self.HorizontalMotion.get()

    def HorizontalToggle(self):
        self.HorizontalMotion.toggle()
    
    def VerticalToggle(self):
        self.VerticalMotion.toggle()

    def InNOutCmd(self) -> InNOutCommand:
        return InNOutCommand(self)
    
    def IntakeCmd(self) -> IntakeCommand:
        return IntakeCommand(self)
    
class InNOutCommand(commands2.Command):
    def __init__(self, intake: Intake):
        self.addRequirements(intake)
        self.intake = intake
        self.forward = wpilib.DoubleSolenoid.Value.kForward
        self.backward = wpilib.DoubleSolenoid.Value.kReverse
        self.timer = wpilib.Timer()

    def UpdateStates(self):
        self.horizontal_state = self.intake.HorizontalState()
        self.vertical_state = self.intake.VerticalState()

    def execute(self):
        self.UpdateStates()
        if self.horizontal_state == self.forward and self.vertical_state == self.forward:
            self.intake.VerticalToggle()
            self.timer.start()
            self.intake.deploying = 0
        elif self.horizontal_state == self.backward and self.vertical_state == self.backward:
            self.intake.HorizontalToggle()
            self.timer.start()
            self.intake.deploying = 1

    def isFinished(self) -> bool:
        self.UpdateStates()
        wait_time = self.timer.hasElapsed(2)
        if self.horizontal_state == self.forward and self.vertical_state == self.backward and wait_time:
            if self.intake.deploying == 1:
                self.intake.VerticalToggle()
                return True
            elif self.intake.deploying == 0:
                self.intake.HorizontalToggle()
                return True
        
    def end(self, interrupted):
        self.timer.stop()
        self.timer.reset()

        self.intake.deploying = -1
    
class IntakeCommand(commands2.Command):
    def __init__(self, intake: Intake):
        self.addRequirements(intake)
        self.intake = intake

    def execute(self):
        self.intake.IntakeMotor.set(-1)

    def end(self, interrupted):
        self.intake.IntakeMotor.set(0)