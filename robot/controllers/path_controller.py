from os import path
import pickle

from magicbot import StateMachine, state
import pathfinder as pf
from pathfinder.followers import DistanceFollower

from components import drivetrain
from controllers import angle_controller

MAX_VELOCITY = 3.66


class PathController(StateMachine):

    drivetrain = drivetrain.Drivetrain
    angle_controller = angle_controller.AngleController

    def setup(self):
        self.points = None
        self.finished = False

        self.left = DistanceFollower([])
        self.right = DistanceFollower([])

        # The first argument is the proportional gain. Usually this will be quite high
        # The second argument is the integral gain. This is unused for motion profiling
        # The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
        # The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the
        #      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
        # The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
        self.left.configurePIDVA(1, 0.0, 0.0, 1 / MAX_VELOCITY, 0)
        self.right.configurePIDVA(1, 0.0, 0.0, 1 / MAX_VELOCITY, 0)

    def set(self, _path):
        self.path = _path
        self.finished = False
        print('set path', _path)

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

        self.left.setTrajectory(left_traj)
        self.right.setTrajectory(right_traj)

        self.next_state('exec_path')

    @state
    def exec_path(self):
        print('current pos', self.drivetrain.get_left_encoder_meters(), self.drivetrain.get_right_encoder_meters())
        try:
            l_o = self.left.calculate(self.drivetrain.get_left_encoder_meters())
            r_o = self.right.calculate(self.drivetrain.get_right_encoder_meters())
        except Exception:
            return
        print(l_o, r_o)

        gyro_heading = self.angle_controller.get_angle()
        desired_heading = pf.r2d(self.left.getHeading())


        angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = 0.025 * angleDifference

        # print('gyro', gyro_heading, 'desired', desired_heading, 'anglediff',angleDifference, 'turn', turn)

        self.drivetrain.manual_drive(l_o + turn, r_o - turn)

        if self.left.isFinished() and self.right.isFinished():
            print('both finished')
            self.finished = True

    def on_disable(self):
        self.drivetrain.set_manual_mode(False)
