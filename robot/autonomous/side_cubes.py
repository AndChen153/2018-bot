from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from controllers.angle_controller import AngleController
from controllers.trajectory_controller import TrajectoryController
from components.drivetrain import Drivetrain
from components.elevator import Elevator, ElevatorPosition
from components.grabber import Grabber
from components.field import Field, SwitchState


class SideAutonomous(StatefulAutonomous):

    DEFAULT = False
    ONE_CUBE_ONLY = False
    start_side = None

    angle_controller = AngleController
    trajectory_controller = TrajectoryController
    drivetrain = Drivetrain
    elevator = Elevator
    grabber = Grabber
    field = Field

    @state(first=True)
    def prepare_to_start(self):
        self.end_after_trajectory = False

        self.elevator.release_lock()
        self.elevator.raise_to_switch()
        self.trajectory_controller.reset()
        switch_side = self.field.get_switch_side()
        if switch_side is not None:
            if switch_side == SwitchState.LEFT:
                self.path_key = 'left'
                self.sign = 1
            else:
                self.path_key = 'right'
                self.sign = -1

            if switch_side == self.start_side:
                if self.ONE_CUBE_ONLY:
                    self.trajectory_controller.push(path='side_forward')
                    self.trajectory_controller.push(rotate=90 * self.sign)
                    self.trajectory_controller.push(position=20, timeout=0.5)
                else:
                    self.trajectory_controller.push(position=228)
                    self.trajectory_controller.push(rotate=90 * self.sign)
                    self.trajectory_controller.push(position=60)
                    self.trajectory_controller.push(rotate=90 * self.sign)
                    self.trajectory_controller.push(position=20, timeout=0.5)
            else:
                self.elevator.lower_to_ground()
                self.trajectory_controller.push(path='side_forward')
                self.trajectory_controller.push(path='side_return_%s'
                                                     % self.path_key,
                                                reverse=True)
                self.end_after_trajectory = True
            self.next_state('execute_trajectory')

    @state
    def execute_trajectory(self):
        if self.trajectory_controller.is_finished():
            if self.end_after_trajectory:
                self.done()
            else:
                self.next_state('deposit')

    @timed_state(duration=0.75, next_state='back_up_to_hunt')
    def deposit(self):
        self.grabber.deposit()

    @state
    def back_up_to_hunt(self):
        if self.ONE_CUBE_ONLY:
            self.done()
            return
        self.trajectory_controller.push(position=-20)
        self.trajectory_controller.push(position=45 * self.sign)
        self.trajectory_controller.push(position=10)
        self.next_state('execute_hunt_trajectory')

    @state
    def execute_hunt_trajectory(self):
        self.elevator.lower_to_ground()
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.next_state('intake_second_cube')

    @timed_state(duration=1, next_state='rotate_back')
    def intake_second_cube(self):
        self.grabber.intake()

    def rotate_back(self):
        self.elevator.raise_to_switch()
        self.trajectory_controller.push(position=-10)
        self.trajectory_controller.push(position=-45 * self.sign)
        self.trajectory_controller.push(position=10, timeout=0.5)
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.SWITCH):
            self.next_state('execute_move_trajectory')

    @state
    def execute_move_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit_second_cube')

    @timed_state(duration=3)
    def deposit_second_cube(self):
        self.grabber.deposit()


class TwoCubeLeft(SideAutonomous):

    MODE_NAME = 'Left - Two Cubes'
    start_side = SwitchState.LEFT


class TwoCubeRight(SideAutonomous):

    MODE_NAME = 'Right - Two Cubes'
    start_side = SwitchState.RIGHT


class OneCubeLeft(SideAutonomous):

    MODE_NAME = 'Left - One Cube'
    ONE_CUBE_ONLY = True
    start_side = SwitchState.LEFT


class OneCubeRight(SideAutonomous):

    MODE_NAME = 'Right - One Cube'
    ONE_CUBE_ONLY = True
    start_side = SwitchState.RIGHT
