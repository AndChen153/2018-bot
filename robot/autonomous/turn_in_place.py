from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from controllers.angle_controller import AngleController
from components.drivetrain import Drivetrain

WAIT_PERIOD = 1  # Seconds to wait before turning
ANGLE = 90  # Degrees to turn in place


class TurnInPlace(StatefulAutonomous):

    MODE_NAME = 'Turn In Place'
    DEFAULT = False

    angle_controller = AngleController
    drivetrain = Drivetrain

    @state(first=True)
    def prepare_to_start(self):
        self.angle_controller.reset_angle()
        self.next_state('turn_wait')

    @timed_state(duration=WAIT_PERIOD, next_state='turn')
    def turn_wait(self):
        self.drivetrain.differential_drive(0)

    @state
    def turn(self):
        self.angle_controller.align_to(ANGLE)
        if self.angle_controller.is_aligned():
            self.done()
