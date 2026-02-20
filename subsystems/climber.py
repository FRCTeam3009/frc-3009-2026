import rev
import wpimath.system.plant
import commands2
import ntcore
import can_ids

SPEED = 1.0

class Climber(commands2.Subsystem):
    def __init__(self):
        self.climber_motor = rev.SparkMax(can_ids.climber, rev.SparkLowLevel.MotorType.kBrushless)
        self.climber_motor_sim = rev.SparkMaxSim(self.climber_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.climber_speed = 1.0

        self.upper_limit = 100
        self.lower_limit = 0
        self.auto_limit = 50

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.climber_table = self.ntcore_instance.getTable("Climber")
        self.climber_topic = self.climber_table.getFloatTopic("MotorSpeed")
        self.motor_speed_publish = self.climber_topic.publish()
        self.motor_speed_publish.set(self.climber_speed)
        self.motor_speed_subscribe = self.climber_topic.subscribe(self.climber_speed)

    def climber_movement(self, speed: float):
        self.climber_motor.set(speed)
        self.climber_motor_sim.setAppliedOutput(speed)

    def get_position(self):
        return self.climber_motor.getEncoder().getPosition()
    
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

class MoveClimberCommand(commands2.Command):
    def __init__(self, climber: Climber, speed: float):
        self.climber = climber
        self.speed = speed
        self.upper_limit = self.climber.upper_limit
        self.addRequirements(self.climber)

    def execute(self):
        self.climber.climber_speed = self.climber.motor_speed_subscribe.get()
        self.climber.climber_movement(self.speed * self.climber.climber_speed)

    def isFinished(self) -> bool:
        if self.climber.climber_motor.getEncoder().getPosition() >= self.upper_limit and self.speed > 0:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.climber.climber_movement(0)

class UpsiesCommand(commands2.Command):
    def __init__(self, climber: Climber):
        self.climber = climber
        self.limit = self.climber.auto_limit

    def execute(self):
        self.climber.climber_speed = self.climber.motor_speed_subscribe.get()
        self.climber.climber_movement(-1 * self.climber.climber_speed)

    def isFinished(self) -> bool:
        if self.climber.climber_motor.getEncoder().getPosition() <= self.limit:
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
        self.pos = self.climber.climber_motor.getEncoder().getPosition()

    def execute(self):
        if self.climber.climber_motor.getEncoder().getPosition() < (self.pos - 5):
            self.climber.climber_speed = self.climber.motor_speed_subscribe.get()
            self.climber.climber_movement(self.climber.climber_speed * 0.5)
        if self.climber.climber_motor.getEncoder().getPosition() >= self.pos:
            self.climber.climber_movement(0)