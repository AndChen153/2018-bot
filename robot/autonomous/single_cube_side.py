from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from controllers.trajectory_controller import TrajectoryController
from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.grabber import Grabber
from components.field import Field, SwitchState


class SingleCubeSide(StatefulAutonomous):

    DEFAULT = False
    start_side = None

    trajectory_controller = TrajectoryController
    drivetrain = Drivetrain
    elevator = Elevator
    grabber = Grabber
    field = Field

    @state(first=True)
    def prepare_to_start(self):
        self.elevator.release_lock()
        self.elevator.raise_to_switch()
        self.trajectory_controller.reset()
        switch_side = self.field.get_switch_side()
        if switch_side is not None:
            sign = 1 if self.start_side == SwitchState.LEFT else -1
            if switch_side == self.start_side:
                self.trajectory_controller.push(position=170)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=10, timeout=3)
            else:
                self.trajectory_controller.push(position=228)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=80)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=10, timeout=3)
            self.next_state('execute_trajectory')

    @state
    def execute_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit')

    @timed_state(duration=3)
    def deposit(self):
        self.grabber.deposit()


class SingleCubeLeft(SingleCubeSide):

    MODE_NAME = 'Single Cube From Left'
    start_side = SwitchState.LEFT


class SingleCubeRight(SingleCubeSide):

    MODE_NAME = 'Single Cube From Right'
    start_side = SwitchState.RIGHT
