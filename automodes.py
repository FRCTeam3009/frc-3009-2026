import pathplannerlib.path
import pathplannerlib.auto
import wpimath.units
import commands2
import subsystems.command_swerve_drivetrain
import subsystems.drive_robot_relative
import ntcore
import wpilib
import commands2
import subsystems.shooter
from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d
import subsystems.shooter
import math
import subsystems.climber
import subsystems.autoEndHubRB

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


auto_movement = 1
# side_start is middle of the ramp
side_start = 0.927
speed_auto = 0.50

def auto_move_back() -> float:
    return math.sqrt(math.pow(auto_movement, 2) - math.pow(side_start, 2))

back_move_sideways = auto_move_back()

def get_rotation() -> float:
    print(math.degrees(math.asin(side_start / auto_movement)))
    return math.asin(side_start / auto_movement)

rot_auto = get_rotation()

def start_pose(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain) -> Pose2d:
    starting_position = Pose2d(5, 5, 0)
    drivetrain.reset_pose(starting_position)
    return starting_position

class AutoCommand():
    drive_to_pose = 1
    drive_shoot = 2
    wait = 3
    def __init__(self, position : Pose2d, type : int, aprtag: int):
        self.position = position
        self.type = type
        self.april_tag_id = aprtag

def pathplanner_constraints() -> pathplannerlib.path.PathConstraints: 
    # Create the constraints to use while pathfinding
    return pathplannerlib.path.PathConstraints(
        2.0,
        2.0,
        wpimath.units.rotationsToRadians(0.75),
        wpimath.units.rotationsToRadians(0.75), 
    )

def noob_auto_drive_straight_forward(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_command(drivetrain, wpimath.units.meters(1.5), subsystems.drive_robot_relative.NORMAL_SPEED, 0)

def sit() -> commands2.Command:
    return commands2.Command()

def move_shoot_center(
        shooter: subsystems.shooter.Shooter,
        drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain
    ) -> commands2.Command:
    sp = start_pose(drivetrain)
    transform = sp.transformBy(wpimath.geometry.Transform2d(-auto_movement, 0, 0))
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(transform))
    cmds.addCommands(shoot_fuel(shooter).withTimeout(5.0))
    return cmds

def move_shoot_right(
        shooter: subsystems.shooter.Shooter,
        drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain
    ) -> commands2.Command:
    sp = start_pose(drivetrain)
    transform = sp.transformBy(wpimath.geometry.Transform2d(-auto_movement, 0, rot_auto))
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(transform))
    cmds.addCommands(shoot_fuel(shooter).withTimeout(5.0))
    return cmds

def move_shoot_left(
        shooter: subsystems.shooter.Shooter,
        drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain
    ) -> commands2.Command:
    sp = start_pose(drivetrain)
    transform = sp.transformBy(wpimath.geometry.Transform2d(-auto_movement, 0, -rot_auto))
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(transform))
    cmds.addCommands(shoot_fuel(shooter).withTimeout(5.0))
    return cmds

def blue_climb(
        climber: subsystems.climber.Climber,
    ) -> commands2.Command:
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(positions[1])) # change pos to correct pos
    cmds.addCommands(climb_set_up(climber))
    pos: Pose2d = positions[1]
    transform = pos.transformBy(wpimath.geometry.Transform2d(0, 0, 0)) #transform forward to climb
    cmds.addCommands(drive_to_pose(transform))
    cmds.addCommands(climb_up(climber))
    return cmds

def drive_to_pose(position: Pose2d) -> commands2.Command:
    return pathplannerlib.auto.AutoBuilder.pathfindToPose(
            position,
            pathplanner_constraints(),
            0.0,
        )

def shoot_fuel(shooter: subsystems.shooter.Shooter) -> commands2.Command:
    def shooter_speed():
        return shooter.shooter_speed
    return shooter.fire_cmd(shooter_speed)

def climb_up(climber: subsystems.climber.Climber) -> commands2.Command:
    return climber.upsies()

def climb_set_up(climber: subsystems.climber.Climber) -> commands2.Command:
    return climber.move_cmd(climber.climber_speed)

class AutoDashboard():
    AUTO_SIT = "sit"
    AUTO_NOOB_FORWARD = "noob_forward"
    AUTO_MOVE_SHOOT_CENTER = "move_shoot_center"
    AUTO_MOVE_SHOOT_LEFT = "move_shoot_left"
    AUTO_MOVE_SHOOT_RIGHT = "move_shoot_right"

    auto_mode_list = [
        AUTO_SIT,
        AUTO_NOOB_FORWARD,
        AUTO_MOVE_SHOOT_CENTER,
        AUTO_MOVE_SHOOT_LEFT,
        AUTO_MOVE_SHOOT_RIGHT,
    ]
            
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable("auto modes")
        self.optionstopic = self.table.getStringArrayTopic("options")
        self.options_publisher = self.optionstopic.publish()
        self.options_publisher.set(list(self.auto_mode_list))

        self.selectedtopic = self.table.getStringTopic("selected")
        self.selected_publisher = self.selectedtopic.publish()
        self.selected_publisher.set("sit")
        self.selected_subscriber = self.selectedtopic.subscribe("sit")
        self.current_auto = self.selected_subscriber.get() 

        self.hub_active_table = self.nt_instance.getTable("hub")
        self.hub_active_publish_topic = self.hub_active_table.getBooleanTopic("hubActive")
        self.hub_active_publish = self.hub_active_publish_topic.publish()
        self.hub_active_publish.set(False)       

    def update(self):
        self.options_publisher.set(list(self.auto_mode_list))
        self.current_auto = self.selected_subscriber.get()
        hubState = subsystems.autoEndHubRB.is_hub_active()
        self.hub_active_publish.set(hubState)

    def get_current_auto_builder(self, drivetrain, shooter):
        if self.current_auto == AutoDashboard.AUTO_NOOB_FORWARD:
            return noob_auto_drive_straight_forward(drivetrain)
        elif self.current_auto == AutoDashboard.AUTO_MOVE_SHOOT_CENTER:
            return move_shoot_center(shooter, drivetrain)
        elif self.current_auto == AutoDashboard.AUTO_MOVE_SHOOT_LEFT:
            return move_shoot_left(shooter, drivetrain)
        elif self.current_auto == AutoDashboard.AUTO_MOVE_SHOOT_RIGHT:
            return move_shoot_right(shooter, drivetrain)
        else:
            return sit()
    

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