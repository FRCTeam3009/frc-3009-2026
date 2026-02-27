import subsystems.autoEndHubRB

def test_timeremaining():
    # Automode just passes through
    output = subsystems.autoEndHubRB.timeremaining(-4, True)
    assert(output == -4)

    # Teleop start, transition period
    output = subsystems.autoEndHubRB.timeremaining(140, False)
    assert(output == 10)

    output = subsystems.autoEndHubRB.timeremaining(110, False)
    assert(output == 5)
    
    output = subsystems.autoEndHubRB.timeremaining(89, False)
    assert(output == 9)

    output = subsystems.autoEndHubRB.timeremaining(54, False)
    assert(output == 24)

    output = subsystems.autoEndHubRB.timeremaining(12, False)
    assert(output == 12)


def run_tests():
    test_timeremaining()

if __name__ == "__main__":
    run_tests()
