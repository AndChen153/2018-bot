from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

from controllers.angle_controller import AngleController
from controllers.trajectory_controller import TrajectoryController
from controllers.cube_hunter_controller import CubeHunterController
from components.drivetrain import Drivetrain
from components.elevator import Elevator, ElevatorPosition
from components.grabber import Grabber
from components.field import Field, SwitchState


class ThreeCube(StatefulAutonomous):

    MODE_NAME = 'OLD Three Cube From Center'
    DEFAULT = False

    angle_controller = AngleController
    trajectory_controller = TrajectoryController
    drivetrain = Drivetrain
    elevator = Elevator
    grabber = Grabber
    field = Field
    cube_hunter_controller = CubeHunterController

    @state(first=True)
    def prepare_to_start(self):
        self.elevator.release_lock()
        self.elevator.raise_to_switch()
        self.trajectory_controller.reset()
        switch_side = self.field.get_switch_side()
        if switch_side:
            self.sign = 1 if switch_side == SwitchState.RIGHT else -1
            self.trajectory_controller.push(rotate=25 * self.sign)
            self.trajectory_controller.push(position=110)
            self.trajectory_controller.push(rotate=-25 * self.sign)
            self.trajectory_controller.push(position=5, timeout=2)
            self.next_state('execute_trajectory')

    @state
    def execute_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit')

    @timed_state(duration=2, next_state='go_down')
    def deposit(self):
        self.grabber.deposit()

    @state
    def go_down(self):
        self.trajectory_controller.push(position=-5)
        self.next_state('lower')

    @state
    def lower(self):
        self.elevator.lower_to_ground()
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.next_state('rotate_to_cube')

    @state
    def rotate_to_cube(self):
        self.trajectory_controller.push(rotate=-90 * self.sign)
        self.next_state('execute_rotate_trajectory')

    @state
    def execute_rotate_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.pre_hunting_angle = self.angle_controller.get_angle()
            self.cube_hunter_controller.reset()
            self.next_state('find_second_cube')

    @state
    def find_second_cube(self):
        self.cube_hunter_controller.seek()
        if self.cube_hunter_controller.is_acquired():
            self.next_state('intake_second_cube')

    @timed_state(duration=1, next_state='rotate_back')
    def intake_second_cube(self):
        self.grabber.intake()

    @state
    def rotate_back(self):
        self.elevator.raise_to_switch()
        self.angle_controller.align_to(self.pre_hunting_angle)
        if self.angle_controller.is_aligned() and \
                self.elevator.is_at_position(ElevatorPosition.SWITCH):
            self.trajectory_controller.push(position=-20)
            self.trajectory_controller.push(rotate=90 * self.sign)
            self.trajectory_controller.push(position=5, timeout=2)
            self.next_state('execute_move_trajectory')

    @state
    def execute_move_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit_second_cube')

    @timed_state(duration=3, next_state='go_down_for_third_cube')
    def deposit_second_cube(self):
        self.grabber.deposit()

    @state
    def go_down_for_third_cube(self):
        self.trajectory_controller.push(position=-20)
        self.next_state('lower_for_third_cube')

    @state
    def lower_for_third_cube(self):
        self.elevator.lower_to_ground()
        if self.trajectory_controller.is_finished() and \
                self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.next_state('rotate_to_third_cube')

    @state
    def rotate_to_third_cube(self):
        self.trajectory_controller.push(rotate=-90 * self.sign)
        self.next_state('execute_third_rotate_trajectory')

    @state
    def execute_third_rotate_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.pre_hunting_angle = self.angle_controller.get_angle()
            self.cube_hunter_controller.reset()
            self.next_state('find_third_cube')

    @state
    def find_third_cube(self):
        self.cube_hunter_controller.seek()
        if self.cube_hunter_controller.is_acquired():
            self.next_state('intake_third_cube')

    @timed_state(duration=1, next_state='rotate_back_from_third')
    def intake_third_cube(self):
        self.grabber.intake()

    @state
    def rotate_back_from_third(self):
        self.elevator.raise_to_switch()
        self.angle_controller.align_to(self.pre_hunting_angle)
        if self.angle_controller.is_aligned() and \
                self.elevator.is_at_position(ElevatorPosition.SWITCH):
            self.trajectory_controller.push(position=-20)
            self.trajectory_controller.push(rotate=90 * self.sign)
            self.trajectory_controller.push(position=20, timeout=2)
            self.next_state('execute_third_move_trajectory')

    @state
    def execute_third_move_trajectory(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit_third_cube')

    @timed_state(duration=3)
    def deposit_third_cube(self):
        self.grabber.deposit()
