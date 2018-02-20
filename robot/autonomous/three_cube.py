from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from controllers.angle_controller import AngleController
from controllers.trajectory_controller import TrajectoryController
from controllers.cube_hunter_controller import CubeHunterController
from controllers.grabber_auto_controller import GrabberAutoController
from components.drivetrain import Drivetrain
from components.elevator import Elevator, ElevatorPosition
from components.grabber import Grabber
from components.field import Field, SwitchState


class ThreeCube(StatefulAutonomous):

    ONE_CUBE_ONLY = False
    TWO_CUBE_ONLY = False

    angle_controller = AngleController
    trajectory_controller = TrajectoryController
    drivetrain = Drivetrain
    elevator = Elevator
    grabber = Grabber
    field = Field
    cube_hunter_controller = CubeHunterController
    grabber_auto_controller = GrabberAutoController

    @state(first=True)
    def prepare_to_start(self):
        self.elevator.release_lock()
        self.elevator.raise_to_switch()
        self.trajectory_controller.reset()
        self.grabber_auto_controller.disable()
        switch_side = self.field.get_switch_side()
        if switch_side:
            self.sign = 1 if switch_side == SwitchState.RIGHT else -1
            self.trajectory_controller.push(position=15)
            self.trajectory_controller.push(rotate=35 * self.sign)
            self.trajectory_controller.push(position=90)
            self.trajectory_controller.push(rotate=-35 * self.sign)
            self.trajectory_controller.push(position=10, timeout=0.5)
            self.next_state('execute_trajectory')

    @state
    def execute_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit')

    @timed_state(duration=1, next_state='go_down')
    def deposit(self):
        self.grabber.deposit()

    @state
    def go_down(self):
        if self.ONE_CUBE_ONLY:
            self.done()
            return
        self.elevator.lower_to_ground()
        self.trajectory_controller.reset()
        self.trajectory_controller.push(position=-59)
        self.trajectory_controller.push(rotate=-35 * self.sign)
        self.trajectory_controller.push(position=40, timeout=3)
        self.next_state('lower')

    @state
    def lower(self):
        if self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.grabber.intake()
            if self.trajectory_controller.is_finished():
                self.next_state('intake_second_cube')

    @timed_state(duration=0.25, next_state='rotate_back')
    def intake_second_cube(self):
        self.grabber.intake()

    @state
    def rotate_back(self):
        self.elevator.raise_to_switch()
        self.trajectory_controller.reset()
        self.trajectory_controller.push(position=-45)
        self.trajectory_controller.push(rotate=35 * self.sign)
        self.trajectory_controller.push(position=67, timeout=4)
        self.next_state('execute_move_trajectory')

    @state
    def execute_move_trajectory(self):
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.SWITCH):
            self.next_state('deposit_second_cube')

    @timed_state(duration=1, next_state='go_down_for_third_cube')
    def deposit_second_cube(self):
        self.grabber.deposit()

    @state
    def go_down_for_third_cube(self):
        if self.TWO_CUBE_ONLY:
            self.done()
            return
        self.elevator.lower_to_ground()
        self.trajectory_controller.reset()
        self.trajectory_controller.push(position=-50)
        self.trajectory_controller.push(rotate=-40 * self.sign)
        self.trajectory_controller.push(position=30)
        self.next_state('lower_for_third_cube')

    @state
    def lower_for_third_cube(self):
        if self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.grabber.intake()
            if self.trajectory_controller.is_finished():
                self.next_state('intake_third_cube')

    @timed_state(duration=0.5, next_state='rotate_back_from_third')
    def intake_third_cube(self):
        self.grabber.intake()

    @state
    def rotate_back_from_third(self):
        self.elevator.raise_to_switch()
        self.trajectory_controller.reset()
        self.trajectory_controller.push(position=-30)
        self.trajectory_controller.push(rotate=40 * self.sign)
        self.trajectory_controller.push(position=30, timeout=3)
        self.next_state('execute_third_move_trajectory')

    @state
    def execute_third_move_trajectory(self):
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.SWITCH):
            self.next_state('deposit_third_cube')

    @timed_state(duration=1)
    def deposit_third_cube(self):
        self.grabber.deposit()


class ThreeCubeReal(ThreeCube):
    MODE_NAME = 'Three Cube From Center'
    DEFAULT = False
