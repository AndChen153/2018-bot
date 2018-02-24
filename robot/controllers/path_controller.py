from os import path
import pickle

import hal
from magicbot import StateMachine, state
import pathfinder as pf
from pathfinder.followers import DistanceFollower

from components import drivetrain
from controllers import angle_controller

if hal.HALIsSimulation():
    import pyfrc

MAX_VELOCITY = 3.66
CONV_Y = 2.5
CONV_X = 5.5


class PathController(StateMachine):

    drivetrain = drivetrain.Drivetrain
    angle_controller = angle_controller.AngleController

    def setup(self):
        self.points = None
        self.finished = False

        self.left = DistanceFollower([])
        self.right = DistanceFollower([])

        # Argument format:
        # - P gain
        # - Integral gain (0)
        # - Derivative gain (tracking)
        # - Velocity ratio (1/max velo in trajectory config)
        # - Accel gain
        self.left.configurePIDVA(1, 0.0, 0.0, 1 / MAX_VELOCITY, 0)
        self.right.configurePIDVA(1, 0.0, 0.0, 1 / MAX_VELOCITY, 0)

    def set(self, _path):
        self.path = _path
        self.finished = False

    def is_finished(self):
        return self.finished

    def run(self):
        self.engage()

    @state(first=True)
    def prepare(self):
        self.drivetrain.shift_low_gear()
        self.drivetrain.set_manual_mode(True)
        self.drivetrain.reset_position()
        self.angle_controller.reset_angle()

        basepath = path.dirname(__file__)
        traj_path = path.abspath(path.join(basepath, '../paths', self.path))
        left_traj = pickle.load(open(traj_path + '-l', 'rb'))
        right_traj = pickle.load(open(traj_path + '-r', 'rb'))

        self.left.reset()
        self.right.reset()
        self.left.setTrajectory(left_traj)
        self.right.setTrajectory(right_traj)

        if hal.HALIsSimulation():
            renderer = pyfrc.sim.get_user_renderer()
            if renderer:
                renderer.draw_pathfinder_trajectory(
                    left_traj, color='blue')
                renderer.draw_pathfinder_trajectory(
                    right_traj, color='pink')

                # renderer.draw_pathfinder_trajectory(
                #     left_traj, scale=(CONV_X, -CONV_Y), offset=(-1.5, -0.5))
                # renderer.draw_pathfinder_trajectory(
                #     right_traj, scale=(4, -4))

        self.next_state('exec_path')

    @state
    def exec_path(self):
        print('[path controller] [current] L: %s; R: %s' %
              (self.drivetrain.get_left_encoder_meters(),
               self.drivetrain.get_right_encoder_meters()))

        try:
            l_o = self.left.calculate(
                self.drivetrain.get_left_encoder_meters())
            r_o = self.right.calculate(
                self.drivetrain.get_right_encoder_meters())
        except Exception:
            return

        print('[path controller] [calculated] L: %s; R: %s' % (l_o, r_o))

        gyro_heading = self.angle_controller.get_angle()
        desired_heading = pf.r2d(self.left.getHeading())

        angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = 0.025 * angleDifference

        self.drivetrain.manual_drive(l_o + turn, r_o - turn)

        if self.left.isFinished() and self.right.isFinished():
            self.stop()
            self.finished = True

    def stop(self):
        self.drivetrain.set_manual_mode(False)
        self.drivetrain.differential_drive(0)
