from os import path
import datetime

from magicbot import StateMachine, state
import pathfinder as pf

from components import drivetrain
from controllers import angle_controller


class PathRecorderController(StateMachine):

    drivetrain = drivetrain.Drivetrain
    angle_controller = angle_controller.AngleController

    def setup(self):
        pass

    def record(self):
        self.engage()

    @state(first=True)
    def prepare(self):
        self.drivetrain.reset_position()
        self.angle_controller.reset_angle()
        self.name = datetime.datetime.now().isoformat()
        self.left = []
        self.right = []
        self._start_tm = 0

        self.next_state('exec_record')

    @state
    def exec_record(self, state_tm=0, initial_call=False):
        if initial_call:
            self._start_tm = state_tm

        self.left.push([
            state_tm - self._start_tm,
            self.drivetrain.get_left_encoder_meters(),
            self.drivetrain.get_left_encoder_velocity_meters()
        ])
        self.right.push([
            state_tm - self._start_tm,
            self.drivetrain.get_right_encoder_meters(),
            self.drivetrain.get_right_encoder_velocity_meters()
        ])

    def done(self):
        print('Recording finished - saving recording')
        # l_traj = [pf.Segment(d[0]) for d in self.left]
        pf.serialize_csv(path.join(path.dirname(__file__),
                                   '../recorded_paths', self.name + '-l.csv'),
                         l_traj)
        pf.serialize_csv(path.join(path.dirname(__file__),
                                   '../recorded_paths', self.name + '-r.csv'),
                         r_traj)
