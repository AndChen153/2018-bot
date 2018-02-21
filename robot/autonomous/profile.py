from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from controllers.path_controller import PathController


from controllers.grabber_auto_controller import GrabberAutoController
from components.elevator import Elevator, ElevatorPosition
from components.grabber import Grabber
from components.field import Field, SwitchState


class MotionProfile(StatefulAutonomous):

    MODE_NAME = 'Center Profile Run'
    DEFAULT = False

    ONE_CUBE_ONLY = True

    path_controller = PathController
    elevator = Elevator
    grabber = Grabber
    field = Field
    grabber_auto_controller = GrabberAutoController

    @state(first=True)
    def begin(self):
        self.elevator.release_lock()
        self.elevator.raise_to_switch()
        self.grabber_auto_controller.disable()
        switch_side = self.field.get_switch_side()
        if switch_side is not None:
            if switch_side == SwitchState.LEFT:
                self.path_controller.set('center_left')
            else:
                self.path_controller.set('center_right')
            self.next_state('drive_to_destination')

    @state()
    def drive_to_destination(self):
        self.path_controller.run()
        if self.path_controller.is_finished():
            self.next_state('deposit')

    @timed_state(duration=1, next_state='acquire_second_cube')
    def deposit(self):
        self.grabber.deposit()

    @state
    def acquire_second_cube(self):
        if self.ONE_CUBE_ONLY:
            self.done()
            return

        self.path_controller.set('center_left_backup')
        self.next_state('backup')

    @state
    def backup(self):
        self.path_controller.run()
        self.elevator.lower_to_ground()
        if self.path_controller.is_finished():
            self.path_controller.set('center_left_to_cube')
            self.next_state('get_second_cube')

    @state
    def get_second_cube(self):
        self.path_controller.run()
        if self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.grabber.intake()
        # if self.path_controller.is_finished():
        #     self.path_controller.set('center_left_to_cube_return')
        #     self.next_state('return_from_second')

    @state
    def return_from_second(self):
        self.path_controller.run()
        if self.path_controller.is_finished():
            self.next_state('deposit_second')

    @timed_state(duration=1)
    def deposit_second(self):
        self.grabber.deposit()
