from ntcore import NetworkTableInstance
import commands2.cmd
import wpimath.geometry
import wpimath.units
import subsystems.command_swerve_drivetrain
import wpimath
import subsystems.drive_robot_relative
import subsystems.limelight_positions
import json
import typing
import phoenix6.utils
import math
import automodes
import wpilib
import subsystems.autoEndHubRB

APRIL_TAG_OFFSET = 0.56
CORAL_OFFSET = wpimath.units.inchesToMeters(-2.5)
HUB_BLUE_POS = wpimath.geometry.Pose2d(
    wpimath.units.inchesToMeters(182.11), 
    wpimath.units.inchesToMeters(158.84), 
    0)
TARGET_DISTANCE = wpimath.units.inchesToMeters(140)

class Limelight(object):
    def __init__(self, name: str, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        # [forward, horizontal, vertical, roll, pitch, yaw, latency, tag count, tag span, average distance, average area]

        self.nt_instance = NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable(name)
        self.hub_table = self.nt_instance.getTable("hub")
        # NOTE Use pose2d_from_botpose() to make this easier to deal with.
        self.botposetopic = self.table.getDoubleArrayTopic("botpose_wpiblue")
        self.botposesub = self.botposetopic.subscribe([])
        self.drive_train = drive_train

        # NOTE Target pose in robot space returns [horizontal, vertical, forward, pitch, yaw, roll].
        # This is NOT CONSISTENT with the bot pose in field space values.
        # Use pose2d_from_targetpose() to make this easier to deal with.
        self.targetpose_botspacetopic = self.table.getDoubleArrayTopic("targetpose_robotspace")
        self.targetpose_botspacesub = self.targetpose_botspacetopic.subscribe([])

        self.stream_topic = self.table.getDoubleTopic("stream")
        self.stream_publish = self.stream_topic.publish()
        # 0 = Side-by-Side
        # 1 = Picture-in-Picture (secondary small)
        # 2 = Picture-in-Picture (primary small)
        self.stream_setting = 0
        self.stream_publish.set(self.stream_setting)

        self.json_topic = self.table.getStringTopic("json")
        self.json_subscribe = self.json_topic.subscribe("")
        self.target_poses = {6: subsystems.limelight_positions.SmoothPosition(),
                            7: subsystems.limelight_positions.SmoothPosition(),
                            8: subsystems.limelight_positions.SmoothPosition(),
                            9: subsystems.limelight_positions.SmoothPosition(),
                            10: subsystems.limelight_positions.SmoothPosition(),
                            11: subsystems.limelight_positions.SmoothPosition(),
                            17: subsystems.limelight_positions.SmoothPosition(),
                            18: subsystems.limelight_positions.SmoothPosition(),
                            19: subsystems.limelight_positions.SmoothPosition(),
                            20: subsystems.limelight_positions.SmoothPosition(),
                            21: subsystems.limelight_positions.SmoothPosition(),
                            22: subsystems.limelight_positions.SmoothPosition(),
                            1: subsystems.limelight_positions.SmoothPosition(),
                            2: subsystems.limelight_positions.SmoothPosition(),
                            12: subsystems.limelight_positions.SmoothPosition(),
                            13: subsystems.limelight_positions.SmoothPosition()}

        self.smooth_botpose = subsystems.limelight_positions.SmoothPosition()

        self.lined_up_topic = self.table.getBooleanTopic("test lined up")
        self.lined_up_publish = self.lined_up_topic.publish()
        self.lined_up_publish.set(False)

        self.bot_pose_topic = self.table.getDoubleArrayTopic("test current bot pose")
        self.bot_pose_publish = self.bot_pose_topic.publish()
        self.bot_pose_publish.set([0.0, 0.0, 0.0])

        self.offset_topic = self.table.getDoubleArrayTopic("lineup_offset")
        self.offset_pub = self.offset_topic.publish()
        self.offset_pub.set([0.0, 0.0, 0.0])

        self.limelight_pose_topic = self.table.getStructTopic("limelight pose", wpimath.geometry.Pose2d)
        self.limelight_pose_publish = self.limelight_pose_topic.publish()
        self.limelight_pose_publish.set(wpimath.geometry.Pose2d())

        self.hub_dist_topic = self.hub_table.getFloatTopic("hubDist")
        self.hub_dist_publish = self.hub_dist_topic.publish()
        self.hub_dist_publish.set(0.0)

        self.within_range_topic = self.hub_table.getBooleanTopic("inRange")
        self.within_range_publish = self.within_range_topic.publish()
        self.within_range_publish.set(False)

        self.hub_angle_topic = self.hub_table.getFloatTopic("hubAngle")
        self.hub_angle_publish = self.hub_angle_topic.publish()
        self.hub_angle_publish.set(0.0)

        self.hub_angle_aligned_topic = self.hub_table.getBooleanTopic("isAligned")
        self.hub_angle_aligned_publish = self.hub_angle_aligned_topic.publish()
        self.hub_angle_aligned_publish.set(False)

        self.hub_in_position_topic = self.hub_table.getBooleanTopic("inPosition")
        self.hub_in_position_publish = self.hub_in_position_topic.publish()
        self.hub_in_position_publish.set(False)

        self.hub_active_publish_topic = self.hub_table.getBooleanTopic("hubActive")
        self.hub_active_publish = self.hub_active_publish_topic.publish()
        self.hub_active_publish.set(False)

        self.hub_timer_topic = self.hub_table.getStringTopic("hubActiveTimer")
        self.hub_timer_publish = self.hub_timer_topic.publish()
        self.hub_timer_publish.set("Infinite Seconds")

        self.should_fire_topic = self.hub_table.getBooleanTopic("shouldFire")
        self.should_fire_publish = self.should_fire_topic.publish()
        self.should_fire_publish.set(False)

        self.should_fire_word_topic = self.hub_table.getStringTopic("shouldFireWord")
        self.should_fire_word_publish = self.should_fire_word_topic.publish()
        self.should_fire_word_publish.set("HOLD")

        self.goalAngle = 0

    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(self.update).repeatedly().ignoringDisable(True)

    def hubTelemetry(self) -> None:
        # Determine whether the Hub is active and how long between states.
        hubState = subsystems.autoEndHubRB.is_hub_active()
        self.hub_active_publish.set(hubState)

        match_time = wpilib.DriverStation.getMatchTime()
        is_auto_enabled = wpilib.DriverStation.isAutonomousEnabled()
        seconds = subsystems.autoEndHubRB.timeremaining(match_time, is_auto_enabled)
        secondsStr = f"{seconds:.2f} Seconds"
        self.hub_timer_publish.set(secondsStr)

        # Get the hub we are targeting (blue or red)
        targetHub = HUB_BLUE_POS
        if automodes.should_mirror():
            targetHub = automodes.mirror_position(targetHub)

        # Find the distance from target
        currentPose = self.drive_train.get_state_copy().pose
        yDistance = targetHub.Y() - currentPose.Y()
        xDistance = targetHub.X() - currentPose.X()
        totalDistance = math.sqrt(math.pow(yDistance, 2) + math.pow(xDistance, 2))
        distanceDisplay = round(wpimath.units.metersToFeet(totalDistance), 2)
        self.hub_dist_publish.set(distanceDisplay)

        # Check if we're in range
        buffer = wpimath.units.inchesToMeters(12.0)
        inRange = False
        if totalDistance < TARGET_DISTANCE + buffer and totalDistance > TARGET_DISTANCE - buffer:
            inRange = True
        self.within_range_publish.set(inRange)

        # Find angle between us and target
        currentRotation = currentPose.rotation().radians()
        measurementPointX = math.cos(currentRotation)
        measurementPointY = math.sin(currentRotation)
        dotProduct = measurementPointX * xDistance + measurementPointY * yDistance
        distanceVar = dotProduct / totalDistance 
        self.goalAngle = math.acos(distanceVar)

        self.hub_angle_publish.set(self.goalAngle * (180 / math.pi))

        # Check are we aligned
        aligned = False
        if abs(self.goalAngle * (180 / math.pi)) <= 5:
            aligned = True

        self.hub_angle_aligned_publish.set(aligned)

        # Are we both in-position and aligned
        inPositionToFire = False
        if aligned and inRange:
            inPositionToFire = True
        self.hub_in_position_publish.set(inPositionToFire)

        # Are we in-position, aligned, and is the hub active.
        # Only then should we fire.
        shouldFire = False
        shouldFireWord = "HOLD"
        if aligned and inRange and hubState:
            shouldFire = True
            shouldFireWord = "FIRE!!!"
        self.should_fire_publish.set(shouldFire)
        self.should_fire_word_publish.set(shouldFireWord)
        

    def update(self):
        # Estimate bot position
        botpose = self.botposesub.get()
        botpose2d = subsystems.limelight_positions.pose2d_from_botpose(botpose)
        if not subsystems.limelight_positions.is_pose2d_zero(botpose2d):
            self.odometry_update(botpose2d)
        self.hubTelemetry()
        # Update AprilTag target positions
        json_str = self.json_subscribe.get()
        if json_str is None or json_str == "":
            for key in self.target_poses.keys():
                val = wpimath.geometry.Pose2d(0.0, 0.0, 0.0)
                self.target_poses[key].append_pose(val)
            return
        
        self.parsed = json.loads(json_str)
        for key in self.target_poses.keys():
            saw_tag = False
            for f in self.parsed["Fiducial"]:
                fid = f["fID"]
                if fid == key:
                    val = subsystems.limelight_positions.pose2d_from_targetpose(f["t6t_rs"])
                    self.target_poses[key].append_pose(val)
                    saw_tag = True
            if not saw_tag:
                val = wpimath.geometry.Pose2d(0.0, 0.0, 0.0)
                self.target_poses[key].append_pose(val)
    
    def odometry_update(self, pose: wpimath.geometry.Pose2d):
        self.drive_train.add_vision_measurement(pose, phoenix6.utils.get_current_time_seconds())
    
    def telemetry(self):
        pose = self.target_poses[19].get_average_pose()
        pose = subsystems.limelight_positions.correct_target_pose(pose)
        bot_pose_target_var : list[typing.SupportsFloat | typing.SupportsIndex]
        bot_pose_target_var = [wpimath.units.metersToInches(pose.X()), 
                                    wpimath.units.metersToInches(pose.Y()), 
                                    pose.rotation().degrees()]
        self.bot_pose_publish.set(bot_pose_target_var)

        botpose = self.botposesub.get()
        botpose2d = subsystems.limelight_positions.pose2d_from_botpose(botpose)
        if not subsystems.limelight_positions.is_pose2d_zero(botpose2d):
            self.limelight_pose_publish.set(botpose2d)

    def reset_pose_command(self, drivetrain):
        return ResetPose(self, drivetrain)
    
    def lock_on(self, drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain) -> commands2.Command:
        return subsystems.drive_robot_relative.drive_command(drivetrain, 0, subsystems.drive_robot_relative.NORMAL_SPEED, self.goalAngle)

class LineupCommand(commands2.Command):
    def __init__(self, drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, limelight: Limelight, april_id: int):
        self.drivetrain = drivetrain
        self.limelight = limelight
        self.april_id = april_id
        self.command = commands2.Command()

    def initialize(self):
        tgt_pose = self.limelight.target_poses[self.april_id]
        if (tgt_pose is None or tgt_pose.is_zero()):
            self.command = subsystems.drive_robot_relative.DriveRobotRelativeCommand(self.drivetrain, wpimath.geometry.Transform2d(0, 0, 0), 0)
            self.command.initialize()
            return
        targetpose = tgt_pose.get_average_pose()
        targetpose = subsystems.limelight_positions.correct_target_pose(targetpose)
        x = targetpose.X() - APRIL_TAG_OFFSET
        y = targetpose.Y() - CORAL_OFFSET
        offset = wpimath.geometry.Transform2d(x, y, targetpose.rotation())
        self.command = subsystems.drive_robot_relative.DriveRobotRelativeCommand(self.drivetrain, offset, subsystems.drive_robot_relative.NORMAL_SPEED)
        self.command.initialize()
    
    def execute(self):
        self.command.execute()
    
    def isFinished(self):
        return self.command.isFinished()
    
class ResetPose(commands2.Command):
    def __init__(self, limelight: Limelight, drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        self.limelight = limelight
        self.drivetrain = drivetrain

    def execute(self):
        botpose = self.limelight.botposesub.get()
        botpose2d = subsystems.limelight_positions.pose2d_from_botpose(botpose)
        if not subsystems.limelight_positions.is_pose2d_zero(botpose2d):
            self.drivetrain.reset_pose(botpose2d)