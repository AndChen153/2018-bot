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
        self.trajectory_controller.reset()
        switch_side = self.field.get_switch_side()
        if switch_side:
            sign = 1 if self.start_side == SwitchState.LEFT else -1
            if switch_side == self.start_side:
                self.trajectory_controller.push(position=36)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=5)
            else:
                self.trajectory_controller.push(position=50)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=40)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=10)
                self.trajectory_controller.push(rotate=90 * sign)
                self.trajectory_controller.push(position=5)
            self.next_state('execute_trajectory')

    @state
    def execute_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit')

    @timed_state(duration=3, next_state='release_lock')
    def deposit(self):
        self.grabber.deposit()

    @timed_state(duration=2)
    def release_lock(self):
        self.elevator.release_lock()


class SingleCubeLeft(SingleCubeSide):

    MODE_NAME = 'Single Cube From Left'
    start_side = SwitchState.LEFT


class SingleCubeRight(SingleCubeSide):

    MODE_NAME = 'Single Cube From Right'
    start_side = SwitchState.RIGHT
