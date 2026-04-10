import commands2
import ntcore
import can_ids
import wpilib
import phoenix6

SPEED = 1.0

class Climber(commands2.Subsystem):
    def __init__(self):
        self.climber_motor = phoenix6.hardware.TalonFX(can_ids.climber)

        self.latches = wpilib.DoubleSolenoid(3, wpilib.PneumaticsModuleType.REVPH, 1, 14) # Claws up/down
        self.arms = wpilib.DoubleSolenoid(3, wpilib.PneumaticsModuleType.REVPH, 2, 13) # Arms open/close
        # self.stabilizer = wpilib.DoubleSolenoid(3, wpilib.PneumaticsModuleType.REVPH, 4, 11) # Stabilizer in/out
        self.latches.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.arms.set(wpilib.DoubleSolenoid.Value.kReverse)
        # self.stabilizer.set(wpilib.DoubleSolenoid.Value.kReverse)

        self.climber_speed = 0.5
        self.climber_speed_auto = -0.5

        #self.upper_limit = -55.92
        self.upper_limit = -50.0
        #self.lower_limit = 0
        self.lower_limit = 5

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
        self.arms.toggle()

    def lower_latch_toggle(self):
        self.latches.toggle()

    def UpperLatchCmd(self) -> UpperLatchCommand:
        return UpperLatchCommand(self)
    
    def LowerLatchCmd(self) -> LowerLatchCommand:
        return LowerLatchCommand(self)
    
    def HoldCmd(self) -> Hold:
        return Hold(self)

class MoveClimberCommand(commands2.Command):
    def __init__(self, climber: Climber, speed: float):
        self.climber = climber
        self.speed = speed
        self.upper_limit = self.climber.upper_limit
        self.addRequirements(self.climber)

    def execute(self):
        self.climber.climber_movement(self.speed * self.climber.climber_speed)

    def isFinished(self) -> bool:
        if self.climber.get_position() <= self.upper_limit and self.speed < 0:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.climber.climber_movement(0)

# Arms open/close
class UpperLatchCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.upper_latch = self.climber.arms

    def execute(self):
        self.climber.upper_latch_toggle()

    def isFinished(self) -> bool:
        return True

# Claws up/down
class LowerLatchCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber

    def execute(self):
        self.climber.lower_latch_toggle()

    def isFinished(self) -> bool:
        return True

class UpsiesCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.addRequirements(climber)
        self.climber = climber
        self.limit = self.climber.lower_limit

    def execute(self):
        self.climber.climber_movement(self.climber.climber_speed)

    def isFinished(self) -> bool:
        if self.climber.get_position() >= self.limit:
            return True
        else:
            return False
        
    def end(self, interrupted: bool):
        self.climber.climber_movement(0)

class Hold(commands2.Command):
    def __init__(self, climber: Climber):
        self.addRequirements(climber)
        self.climber = climber

    def initialize(self):
        self.pos = self.climber.get_position()

    def execute(self):
        if self.climber.climber_motor.get_position().value_as_double < (self.pos - 2):
            self.climber.climber_movement(self.climber.climber_speed * 0.5)
        if self.climber.climber_motor.get_position().value_as_double >= self.pos:
            self.climber.climber_movement(0)
