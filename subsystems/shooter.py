import commands2
import rev
import wpimath
import wpimath.system.plant

class Shooter(commands2.Subsystem):
    def __init__(self):
        self.motor = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))
    
    def move(self, speed: float):
        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)

    def shoot_command(self, speed: float) -> commands2.Command:
        return lambda: (
            self.move(speed)
        )