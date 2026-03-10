import commands2
import ntcore
import can_ids
import wpilib
import phoenix6

SPEED = 1.0

class Climber(commands2.Subsystem):
    def __init__(self):
        self.climber_motor = phoenix6.hardware.TalonFX(can_ids.climber)

        self.latches = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.REVPH, 4, 5)
        self.square_pipe = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.REVPH, 6, 7)
        self.latches.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.square_pipe.set(wpilib.DoubleSolenoid.Value.kReverse)

        self.climber_speed = 0.5

        self.upper_limit = 0
        self.lower_limit = -100
        self.auto_limit = 50

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.climber_table = self.ntcore_instance.getTable("Climber")
        self.climber_topic = self.climber_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.climber_topic.publish()
        self.motor_speed_publish.set(self.climber_speed)
        self.motor_speed_subscribe = self.climber_topic.subscribe(self.climber_speed)

    def climber_movement(self, speed: float):
        self.climber_motor.set(speed)

    def get_position(self):
        return self.climber_motor.get_position().value_as_double
    
    #def telemetry(self):
        #positions = [
        #    self.climber_motor.getEncoder().getPosition(), 
        #    self.climber_motor.getAbsoluteEncoder().getPosition(),
        #    ]
        #self.motor_publish.set(positions)

    def move_cmd(self, speed: float):
        return MoveClimberCommand(self, speed)
    
    def upsies(self):
        return UpsiesCommand(self)
    
    def upper_latch_toggle(self):
        self.square_pipe.toggle()

    def lower_latch_toggle(self):
        self.latches.toggle()

    def UpperLatchCmd(self) -> UpperLatchCommand:
        return UpperLatchCommand(self)
    
    def LowerLatchCmd(self) -> LowerLatchCommand:
        return LowerLatchCommand(self)

class MoveClimberCommand(commands2.Command):
    def __init__(self, climber: Climber, speed: float):
        self.climber = climber
        self.speed = speed
        self.upper_limit = self.climber.upper_limit
        self.addRequirements(self.climber)

    def execute(self):
        self.climber.climber_speed = self.climber.motor_speed_subscribe.get()
        self.climber.climber_movement(self.speed * self.climber.climber_speed)

    '''def isFinished(self) -> bool:
        if self.climber.get_position() >= self.upper_limit and self.speed > 0:
            return True
        else:
            return False'''

    def end(self, interrupted: bool):
        self.climber.climber_movement(0)

class UpperLatchCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.upper_latch = self.climber.square_pipe

    def execute(self):
        self.climber.upper_latch_toggle()

    def isFinished(self) -> bool:
        return True

class LowerLatchCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.lower_latch = self.climber.latches

    def execute(self):
        self.climber.lower_latch_toggle()

    def isFinished(self) -> bool:
        return True

class UpsiesCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.limit = self.climber.auto_limit

    def execute(self):
        self.climber.climber_speed = self.climber.motor_speed_subscribe.get()
        self.climber.climber_movement(-1 * self.climber.climber_speed)

    def isFinished(self) -> bool:
        if self.climber.get_position() <= self.limit:
            return True
        else:
            return False
        
    def end(self, interrupted: bool):
        self.climber.climber_movement(0)

class Hold(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.limit = self.climber.auto_limit

    def initialize(self):
        self.pos = self.climber.get_position()

    def execute(self):
        if self.climber.climber_motor.get_position().value_as_double < (self.pos - 5):
            self.climber.climber_speed = self.climber.motor_speed_subscribe.get()
            self.climber.climber_movement(self.climber.climber_speed * 0.5)
        if self.climber.climber_motor.get_position().value_as_double >= self.pos:
            self.climber.climber_movement(0)