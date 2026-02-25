import wpilib
DriverStation = wpilib.DriverStation
def is_red_active_after_auto() -> bool:
    game_data = DriverStation.getGameSpecificMessage()

    match game_data:
        case "R":
            return False
        case "B":
            return True
        case _:
            # No or invalid game data, assume hub is active.
            return True

def RHubIsActiveAftAut():
    alliance = DriverStation.getAlliance()
    red_active = is_red_active_after_auto()
    if alliance == DriverStation.Alliance.kRed and red_active:
        return True
    elif alliance == DriverStation.Alliance.kBlue and not red_active:
        return True
    else:
        return False
    
def is_hub_active() -> bool:
    alliance = DriverStation.getAlliance()
    # If we have no alliance, we cannot be enabled, therefore no hub.
    if alliance is None:
        return False

    # Hub is always enabled in autonomous.
    if DriverStation.isAutonomousEnabled():
        return True

    # At this point if we're not teleop enabled, there is no hub.
    if not DriverStation.isTeleopEnabled():
        return False

    # We're teleop enabled, compute.
    match_time = DriverStation.getMatchTime()

    # Shift 1 is active for blue if red won auto, or red if blue won auto.
    shift1_active = RHubIsActiveAftAut()

    if match_time > 130:
        return True  # Transition shift, hub is active
    elif match_time > 105:
        # Shift 1
        return shift1_active
    elif match_time > 80:
        # Shift 2
        return not shift1_active
    elif match_time > 55:
        # Shift 3
        return shift1_active
    elif match_time > 30:
        # Shift 4
        return not shift1_active
    else:
        return True  # End game, hub always active
    
def timeremaining(currenttime: float):
    if DriverStation.isAutonomousEnabled():
        return currenttime

    if currenttime > 130:
        return currenttime - 130
    elif currenttime > 105:
        return currenttime - 105
    elif currenttime > 80:
        return currenttime - 80
    elif currenttime > 55:
        return currenttime - 55
    elif currenttime > 30:
        return currenttime - 30
    else:
        return currenttime
