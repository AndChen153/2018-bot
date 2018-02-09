from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from controllers.position_controller import PositionController
from components.drivetrain import Drivetrain

WAIT_PERIOD = 1  # Seconds to wait before driving forward
DISTANCE = 36  # Inches to drive forward


class DriveForward(StatefulAutonomous):

    MODE_NAME = 'Drive Forward'
    DEFAULT = False

    position_controller = PositionController
    drivetrain = Drivetrain

    @state(first=True)
    def prepare_to_start(self):
        self.position_controller.reset_position_and_heading()
        self.next_state('drive_wait')

    @timed_state(duration=WAIT_PERIOD, next_state='drive_forward')
    def drive_wait(self):
        self.drivetrain.differential_drive(0)

    @state
    def drive_forward(self):
        self.position_controller.move_to(DISTANCE)
        if self.position_controller.is_at_location():
            self.done()
