import wpilib
import commands2
import rev
import wpimath.system.plant
import can_ids
import ntcore

class Intake(commands2.Subsystem):
    def __init__(self):
        self.HorizontalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 0, 1)
        self.VerticalMotion = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)
        self.HorizontalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.VerticalMotion.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.IntakeMotor = rev.SparkMax(can_ids.intake, rev.SparkLowLevel.MotorType.kBrushless)
        self.IntakeMotorSim = rev.SparkSim(self.IntakeMotor, wpimath.system.plant.DCMotor.NEO(1))

        # States: -1 = default, 0 = retracting, 1 = deploying
        self.deploying = -1

        intake_motor_speed = -1

        timer_time = 2

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.nttable = self.ntcore_instance.getTable("Intake")
        self.motor_speed_topic = self.nttable.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.motor_speed_topic.publish()
        self.motor_speed_publish.set(intake_motor_speed)
        self.motor_speed_subscribe = self.motor_speed_topic.subscribe(intake_motor_speed)

        self.timer_topic = self.nttable.getFloatTopic("Timer")
        self.timer_publish = self.timer_topic.publish()
        self.timer_publish.set(timer_time)
        self.timer_subscribe = self.timer_topic.subscribe(timer_time)

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
        self.intake.IntakeMotor.set(self.intake.motor_speed_subscribe.get())

    def end(self, interrupted: bool):
        self.intake.IntakeMotor.set(0)