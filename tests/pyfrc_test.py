'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

# Run the default tests, takes ~20 seconds
from pyfrc.tests import * # type: ignore

import subsystems.limelight_positions_tests
import subsystems.intake_test

subsystems.limelight_positions_tests.run_tests()

#subsystems.intake_test.run_tests() # TODO fix tests