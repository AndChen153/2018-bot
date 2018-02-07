# from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state

# from controllers.trajectory_controller import TrajectoryController
# from controllers.grabber_orienter_controller \
#     import GrabberOrienterController, GrabberOrienterSide
# from components.drivetrain import Drivetrain
# from components.elevator import Elevator
# from components.grabber import Grabber
# from components.field import Field, SwitchState


# class TwoCube(StatefulAutonomous):

#     MODE_NAME = 'Two Cube From Center'
#     DEFAULT = False

#     trajectory_controller = TrajectoryController
#     drivetrain = Drivetrain
#     elevator = Elevator
#     grabber = Grabber
#     field = Field
#     grabber_orienter_controller = GrabberOrienterController
#     cube_hunter_controller = CubeHunterController

#     @state(first=True)
#     def prepare_to_start(self):
#         self.trajectory_controller.reset()
#         switch_side = self.field.get_switch_side()
#         if switch_side:
#             sign = 1 if switch_side == SwitchState.RIGHT else -1
#             self.trajectory_controller.push(rotate=30 * sign)
#             self.trajectory_controller.push(position=36)
#             self.trajectory_controller.push(rotate=-30 * sign)
#             self.trajectory_controller.push(position=5)
#             self.next_state('execute_trajectory')

#     @state
#     def execute_trajectory(self):
#         if self.trajectory_controller.is_finished():
#             self.next_state('deposit')

#     @timed_state(duration=3, next_state='release_lock')
#     def deposit(self):
#         self.grabber.deposit()

#     @timed_state(duration=1, next_state='lower')
#     def release_lock(self):
#         self.elevator.release_lock()

#     @timed_state(duration=2, next_state='find_second_cube')
#     def lower(self):
#         self.elevator.lower_to_ground()

#     def find_second_cube(self):
#         self.cube_hunter_controller.seek()
#         self.grabber_orienter_controller.orient(
#             GrabberOrienterSide.FLIPPY)
#         if self.cube_hunter_controller.has_acquired():
#             self.next_state('intake_second_cube')


