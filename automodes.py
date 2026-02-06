import pathplannerlib.path
import pathplannerlib.auto
import wpimath.geometry
import wpimath.units
import commands2
import subsystems.command_swerve_drivetrain
import subsystems.drive_robot_relative
import subsystems.limelight
import ntcore
import wpilib
import commands2
import subsystems.limelight_positions
import subsystems.shooter
from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d
import subsystems.climber
import subsystems.intake
import subsystems.shooter

# Positions for the robot to line up to the April Tags, indexed by April Tag IDs
positions = {}
positions[1] = Pose2d(16.44, 1.03, Rotation2d.fromDegrees(-52)) # Red Coral Pickup Left
positions[2] = Pose2d(16.45, 7.03, Rotation2d.fromDegrees(53)) # Red Coral Pickup Right
positions[3] = Pose2d(11.48, 7.55, Rotation2d.fromDegrees(90)) # Red side, Blue's Algae
positions[6] = Pose2d(14.13, 2.23, Rotation2d.fromDegrees(120)) # Red Coral
positions[7] = Pose2d(14.92, 4.04, Rotation2d.fromDegrees(180)) # Red Coral
positions[8] = Pose2d(14.00, 5.64, Rotation2d.fromDegrees(-120)) # Red Coral
positions[9] = Pose2d(12.16, 5.55, Rotation2d.fromDegrees(-60)) # Red Coral right
positions[10] = Pose2d(11.20, 4.00, Rotation2d.fromDegrees(0)) # Red Coral center
positions[11] = Pose2d(12.05, 2.14, Rotation2d.fromDegrees(60)) # Red Coral left
positions[12] = Pose2d(1.15, 0.99, Rotation2d.fromDegrees(-127)) # Blue Coral Pickup Right
positions[13] = Pose2d(1.17, 7.06, Rotation2d.fromDegrees(127)) # Blue Coral Pickup Left
positions[16] = Pose2d(6.02, 0.52, Rotation2d.fromDegrees(-90)) # Blue Coral
positions[17] = Pose2d(3.67, 2.25, Rotation2d.fromDegrees(60)) # Blue Coral 
positions[18] = Pose2d(2.56, 4.03, Rotation2d.fromDegrees(0)) # Blue Coral
positions[19] = Pose2d(3.41, 5.59, Rotation2d.fromDegrees(-60)) # Blue Coral
positions[20] = Pose2d(5.55, 5.73, Rotation2d.fromDegrees(-120)) # Blue Coral left
positions[21] = Pose2d(6.35, 3.99, Rotation2d.fromDegrees(180)) # Blue Coral center
positions[22] = Pose2d(5.68, 2.14, Rotation2d.fromDegrees(120)) # Blue Coral right

# closer
positions[23] = Pose2d(11.76, 4.21, Rotation2d.fromDegrees(0)) #red
positions[24] = Pose2d(5.79, 3.87, Rotation2d.fromDegrees(180)) #blue
positions[25] = Pose2d(5.00, 5.27, Rotation2d.fromDegrees(-120)) #blueleft
positions[26] = Pose2d(5.00, 2.85, Rotation2d.fromDegrees(120)) #blue right
positions[27] = Pose2d(12.55, 2.82, Rotation2d.fromDegrees(60)) #red left
positions[28] = Pose2d(12.57, 5.24, Rotation2d.fromDegrees(-60)) #red right

class AutoCommand():
    drive_to_pose = 1
    drive_shoot = 2
    wait = 3
    def __init__(self, position : Pose2d, type : int, aprtag: int):
        self.position = position
        self.type = type
        self.april_tag_id = aprtag

def pathplanner_constraints(): 
    # Create the constraints to use while pathfinding
    return pathplannerlib.path.PathConstraints(
        2.0,
        2.0,
        wpimath.units.rotationsToRadians(0.75),
        wpimath.units.rotationsToRadians(0.75), 
    )

def noob_auto_drive_straight_forward(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     climber: subsystems.climber.Climber,
                     shooter: subsystems.shooter.Shooter,
                     intake: subsystems.intake.Intake
                     ) -> commands2.Command:
    
    return subsystems.drive_robot_relative.drive_forward_command(drivetrain, wpimath.units.meters(1.5), subsystems.drive_robot_relative.NORMAL_SPEED)

def sit(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     climber: subsystems.climber.Climber,
                     shooter: subsystems.shooter.Shooter,
                     intake: subsystems.intake.Intake
                     ) -> commands2.Command:
    
    return commands2.Command()

def move_shoot(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     climber: subsystems.climber.Climber,
                     shooter: subsystems.shooter.Shooter,
                     intake: subsystems.intake.Intake
                     ) -> commands2.Command:
    move_to_pose = positions[1]
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(move_to_pose))
    cmds.addCommands(shoot_fuel(shooter).withTimeout(5.0))
    return cmds

def drive_to_pose(position: Pose2d):
    return pathplannerlib.auto.AutoBuilder.pathfindToPose(
            position,
            pathplanner_constraints(),
            0.0,
        )

def shoot_fuel(shooter: subsystems.shooter.Shooter):
    def shooter_speed():
        return shooter.shooter_speed
    return shooter.fire_cmd(shooter_speed)

class AutoDashboard():
    auto_map = {
        "noob_forward": noob_auto_drive_straight_forward,  
        "sit": sit,
        "move_shoot": move_shoot,
       }
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable("auto modes")
        self.optionstopic = self.table.getStringArrayTopic("options")
        self.options_publisher = self.optionstopic.publish()
        self.options_publisher.set(list(self.auto_map.keys()))

        self.selectedtopic = self.table.getStringTopic("selected")
        self.selected_publisher = self.selectedtopic.publish()
        self.selected_publisher.set("sit")
        self.selected_subscriber = self.selectedtopic.subscribe("sit")
        self.current_auto = self.selected_subscriber.get()        

    def update(self):
        self.options_publisher.set(list(self.auto_map.keys()))
        self.current_auto = self.selected_subscriber.get()

    def get_current_auto_builder(self, drivetrain, front_limelight, back_limelight, climber, shooter, intake):
        auto_builder = self.auto_map[self.current_auto]
        return auto_builder(drivetrain, front_limelight, back_limelight, climber, shooter, intake)
    

class WaitCommand(commands2.Command):
    def __init__(self, drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        self.drive_robot_relative = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self.drivetrain = drivetrain
        self.timer = wpilib.Timer()
        
    def execute(self):
        drive_request = lambda: self.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drivetrain.apply_request(drive_request).execute()
        self.timer.start()

    def isFinished(self):
        if self.timer.hasElapsed(9):
            return True
        return False