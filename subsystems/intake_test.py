import subsystems.intake
import wpilib
import time
import commands2

def test_innoutcommand():
    forward =  wpilib.DoubleSolenoid.Value.kForward
    reverse =  wpilib.DoubleSolenoid.Value.kReverse
    i = subsystems.intake.Intake()
    i.timer_publish.set(0.001) # Set small time for test
    c = i.InNOutCmd()
    c.initialize()

    # Starts retracted
    assert(c.isFinished() == False)
    assert(i.HorizontalState() == reverse)
    assert(i.VerticalState() == reverse)
    assert(i.deploying == -1)

    # Deploys
    c.execute()
    assert(c.isFinished() == False)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == reverse)
    assert(c.timer.hasElapsed(0))
    assert(i.deploying == 1)

    # Waits for timer
    c.execute()
    assert(c.isFinished() == False)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == reverse)
    assert(c.timer.hasElapsed(0))
    assert(c.timer.hasElapsed(0.001) == False)
    assert(i.deploying == 1)

    # Once timer elapses, then it finishes
    time.sleep(0.001)
    c.execute()
    assert(i.deploying == 1)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == reverse)
    assert(c.isFinished() == True)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == forward)
    c.end(False)
    assert(i.deploying == -1)

    # Running it again will retract. Starts forward.
    c.initialize()
    assert(c.isFinished() == False)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == forward)
    assert(i.deploying == -1)

    # Pulls up first.
    c.execute()
    assert(c.isFinished() == False)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == reverse)
    assert(i.deploying == 0)

    # Waits for time
    c.execute()
    assert(c.isFinished() == False)
    assert(i.HorizontalState() == forward)
    assert(i.VerticalState() == reverse)
    assert(i.deploying == 0)
    assert(c.timer.hasElapsed(0.001) == False)

    # After time, it finished
    time.sleep(0.001)
    assert(c.timer.hasElapsed(0.001))

    c.execute()
    assert(c.isFinished() == True)
    assert(i.HorizontalState() == reverse)
    assert(i.VerticalState() == reverse)
    c.end(False)
    assert(i.deploying == -1)

def run_tests():
    test_innoutcommand()

if __name__ == "__main__":
    run_tests()