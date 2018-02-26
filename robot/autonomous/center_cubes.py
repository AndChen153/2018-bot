from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from controllers.trajectory_controller import TrajectoryController


from controllers.grabber_auto_controller import GrabberAutoController
from components.elevator import Elevator, ElevatorPosition
from components.grabber import Grabber
from components.field import Field, SwitchState


class CenterAutonomous(StatefulAutonomous):

    ONE_CUBE_ONLY = False

    trajectory_controller = TrajectoryController
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
                self.path_key = 'left'
                self.sign = 1
            else:
                self.path_key = 'right'
                self.sign = -1

            self.trajectory_controller.push(path='center_%s' % self.path_key)
            self.next_state('drive_to_switch')

    @state
    def drive_to_switch(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit')

    @timed_state(duration=0.75, next_state='drive_to_second_cube')
    def deposit(self):
        self.grabber.deposit()

    @state
    def drive_to_second_cube(self):
        if self.ONE_CUBE_ONLY:
            self.done()
            return

        self.trajectory_controller.push(path='center_%s_reverse' %
                                             self.path_key,
                                        reverse=True)
        self.trajectory_controller.push(rotate=45 * self.sign)
        self.trajectory_controller.push(position=40)
        self.next_state('backup')

    @state
    def backup(self):
        self.elevator.lower_to_ground()
        if self.elevator.is_at_position(ElevatorPosition.GROUND):
            self.grabber.intake()

        if self.trajectory_controller.is_finished():
            self.trajectory_controller.push(position=-40)
            self.trajectory_controller.push(rotate=-45 * self.sign)
            self.trajectory_controller.push(path='center_%s_return' %
                                            self.path_key)
            self.next_state('return_from_second_cube')

    @state
    def return_from_second_cube(self):
        if self.trajectory_controller.is_finished():
            self.next_state('deposit_second_cube')

    @timed_state(duration=0.75)
    def deposit_second_cube(self):
        self.grabber.deposit()


class OneCubeCenter(CenterAutonomous):

    MODE_NAME = 'Center - One Cube'
    ONE_CUBE_ONLY = True
    DEFAULT = True


class TwoCubeCenter(CenterAutonomous):

    MODE_NAME = 'Center - Two Cubes'
    DEFAULT = False
