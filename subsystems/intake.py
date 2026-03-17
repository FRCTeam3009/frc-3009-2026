import wpilib
import commands2
import rev
import can_ids
import ntcore
import phoenix6

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

        rollers_motor_speed = -0.35

        timer_time = 0.5

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.nttable = self.ntcore_instance.getTable("Intake")
        self.motor_speed_topic = self.nttable.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.motor_speed_topic.publish()
        self.motor_speed_publish.set(intake_motor_speed)
        self.motor_speed_subscribe = self.motor_speed_topic.subscribe(intake_motor_speed)

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.nttable = self.ntcore_instance.getTable("Intake")
        self.rollers_speed_topic = self.nttable.getFloatTopic("RollerSpeed")
        self.rollers_speed_publish = self.rollers_speed_topic.publish()
        self.rollers_speed_publish.set(rollers_motor_speed)
        self.rollers_speed_subscribe = self.rollers_speed_topic.subscribe(rollers_motor_speed)

        self.timer_topic = self.nttable.getFloatTopic("Timer")
        self.timer_publish = self.timer_topic.publish()
        self.timer_publish.set(timer_time)
        self.timer_subscribe = self.timer_topic.subscribe(timer_time)

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
    
    def IntakeActiveCmd(self) -> IntakeActive:
        return IntakeActive(self)
    
    def telemtry(self):
        self.roller_publish.set(self.is_running)
    
    def RunRollers(self):
        self.rollers.set(self.rollers_speed_subscribe.get())

    def RunRollersBackwards(self):
        self.rollers.set(self.rollers_speed_subscribe.get() * -1)

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
        wait_time = self.timer.hasElapsed(self.intake.timer_subscribe.get())
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

    def execute(self):
        if self.intake.is_running:
            self.intake.IntakeMotor.set(self.intake.motor_speed_subscribe.get())
            self.intake.RunRollers()
        else:
            self.intake.IntakeMotor.set(0)
            self.intake.rollers.set(0)

class StartBool(commands2.Command):
    def __init__(self, intake: Intake):
        self.intake = intake

    def execute(self):
        self.intake.is_running = True

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

