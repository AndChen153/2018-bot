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
    SAME_SIDE_ONLY = False
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
        self.switch_side = self.field.get_switch_side()
        if self.switch_side is not None:

            # Path key
            if self.switch_side == SwitchState.LEFT:
                self.path_key = 'left'
                self.sign = 1
            else:
                self.path_key = 'right'
                self.sign = -1

            # Start side key
            if self.start_side == SwitchState.LEFT:
                self.start_side_key = 'left'
            else:
                self.start_side_key = 'right'

            if self.switch_side == self.start_side:
                self.trajectory_controller.push(path='side_forward')
                self.trajectory_controller.push(rotate=90 * self.sign)
                self.trajectory_controller.push(position=20, timeout=0.5)
            else:
                if self.SAME_SIDE_ONLY:
                    self.elevator.lower_to_ground()
                    self.trajectory_controller.push(path='side_forward')
                    self.trajectory_controller.push(path='side_return_%s'
                                                         % self.path_key,
                                                    reverse=True)
                    self.end_after_trajectory = True
                else:
                    self.trajectory_controller.push(
                        path='side_%s_around_back_to_opposite_side'
                             % self.start_side_key)
                    self.trajectory_controller.push(rotate=-90 * self.sign)
                    self.trajectory_controller.push(position=20, timeout=0.5)

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
        if self.switch_side == self.start_side:
            self.trajectory_controller.push(
                path='side_%s_reverse_to_same_side_cube' % self.start_side_key,
                reverse=True)
            self.trajectory_controller.push(
                path='side_%s_approach_same_side_cube' % self.start_side_key)
            self.trajectory_controller.push(
                path='side_%s_return_approach_same_side' % self.start_side_key,
                reverse=True)

        else:
            raise NotImplementedError

        self.next_state('execute_hunt_trajectory')

    @state
    def execute_hunt_trajectory(self):
        self.elevator.lower_to_ground()
        if self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.grabber.intake()
        if self.trajectory_controller.is_finished():
            self.trajectory_controller.push(
                path='side_%s_deposit_same_side_cube' % self.start_side_key)
            self.next_state('return_to_deposit')

    @state
    def return_to_deposit(self):
        self.elevator.raise_to_switch()
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.SWITCH):
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
    SAME_SIDE_ONLY = False
    start_side = SwitchState.LEFT


class OneCubeRight(SideAutonomous):

    MODE_NAME = 'Right - One Cube'
    ONE_CUBE_ONLY = True
    SAME_SIDE_ONLY = False
    start_side = SwitchState.RIGHT
