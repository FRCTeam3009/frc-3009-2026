import wpilib
import commands2
import rev
import can_ids
import ntcore
import phoenix6
import math

class Intake(commands2.Subsystem):
    def __init__(self):
        self.HorizontalMotion = wpilib.DoubleSolenoid(3, wpilib.PneumaticsModuleType.REVPH, 3, 12)
        self.VerticalMotion = wpilib.DoubleSolenoid(3, wpilib.PneumaticsModuleType.REVPH, 4, 11)
        self.HorizontalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.VerticalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.IntakeMotor = phoenix6.hardware.TalonFX(can_ids.intake)
        self.rollers = rev.SparkMax(can_ids.intake2, rev.SparkLowLevel.MotorType.kBrushless)

        # States: -1 = default, 0 = retracting, 1 = deploying
        self.deploying = -1

        intake_motor_speed = -1

        rollers_motor_speed = -0.4

        timer_time = 0.5

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.nttable = self.ntcore_instance.getTable("Intake")

        self.is_running = False

        self.roller_topic = self.nttable.getBooleanTopic("Rollers")
        self.roller_publish = self.roller_topic.publish()
        self.roller_publish.set(self.is_running)

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
    
    def ChangeBoolCmd(self) -> ChangeBool:
        return ChangeBool(self)
    
    def StartBoolCmd(self) -> StartBool:
        return StartBool(self)
    
    def StopBoolCmd(self) -> StopBool:
        return StopBool(self)
    
    def IntakeActiveCmd(self) -> IntakeActive:
        return IntakeActive(self)
    
    def telemtry(self):
        self.roller_publish.set(self.is_running)
    
    def RunRollers(self, speed):
        self.rollers.set(speed)

    def RunRollersBackwards(self):
        self.rollers.set(self.rollers_motor_speed * -1)
    
    def RunIntakeBackwards(self):
        self.IntakeMotor.set(-1 * self.intake_motor_speed)

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
        if self.intake.deploying != -1:
            # IsFinished will handle the final transition.
            return
        if self.horizontal_state == self.forward and self.vertical_state == self.forward:
            # If deployed, then retract. Start by pulling up first.
            self.intake.VerticalToggle()
            self.timer.start()
            self.intake.deploying = 0
        elif self.horizontal_state == self.backward and self.vertical_state == self.backward:
            # If retracted, then deploy. Start by pushing out first.
            self.intake.HorizontalToggle()
            self.timer.start()
            self.intake.deploying = 1

    def isFinished(self) -> bool:
        self.UpdateStates()
        wait_time = self.timer.hasElapsed(0.5)
        if self.horizontal_state == self.forward and self.vertical_state == self.backward and wait_time:
            # If we're in-between states and enough time has passed, then finish the movement.
            if self.intake.deploying == 1:
                # Finish deploying by pulling down.
                self.intake.VerticalToggle()
                return True
            elif self.intake.deploying == 0:
                # Finish retracting by pulling in.
                self.intake.HorizontalToggle()
                return True
        return False
        
    def end(self, interrupted: bool):
        self.timer.stop()
        self.timer.reset()

        self.intake.deploying = -1
    
class IntakeCommand(commands2.Command):
    def __init__(self, intake: Intake):
        self.addRequirements(intake)
        self.intake = intake
        self.timer_rollers = wpilib.Timer()

    def execute(self):
        self.timer_rollers.start()
        if self.intake.is_running:
            self.intake.IntakeMotor.set(self.intake.intake_motor_speed)
            if math.floor(self.timer_rollers.get() * 3) % 2 == 0:
                self.intake.RunRollers(-0.25)
            else:
                self.intake.RunRollers(-0.5)
        else:
            self.intake.IntakeMotor.set(0)
            self.intake.rollers.set(0)

    def end(self, interrupted: bool):
        self.timer_rollers.stop()
        self.timer_rollers.reset()

class StartBool(commands2.Command):
    def __init__(self, intake: Intake):
        self.intake = intake

    def execute(self):
        self.intake.is_running = True

    def isFinished(self) -> bool:
        return True
    
class StopBool(commands2.Command):
    def __init__(self, intake: Intake):
        self.intake = intake

    def execute(self):
        self.intake.is_running = False

    def isFinished(self) -> bool:
        return True
    
class ChangeBool(commands2.Command):
    def __init__(self, intake: Intake):
        self.intake = intake

    def execute(self):
        self.intake.is_running = not self.intake.is_running

    def isFinished(self) -> bool:
        return True
    

class IntakeActive(commands2.Command):
    def __init__(self, intake: Intake):
        self.addRequirements(intake)
        self.intake = intake

    def execute(self):
        if self.intake.HorizontalState() == wpilib.DoubleSolenoid.Value.kForward:
            self.intake.VerticalToggle()

    def isFinished(self) -> bool:
        return True

