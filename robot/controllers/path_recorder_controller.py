from os import path
import datetime
import pickle

from magicbot import StateMachine, state
import pathfinder as pf

from components import drivetrain
from controllers import angle_controller


class PathRecorderController(StateMachine):

    drivetrain = drivetrain.Drivetrain
    angle_controller = angle_controller.AngleController

    def setup(self):
        self.is_recording = False

    def record(self):
        self.engage()

    @state(first=True)
    def prepare(self):
        print('[PathRecorderController] Beginning recording')
        self.is_recording = True
        self.drivetrain.reset_position()
        self.angle_controller.reset_angle()
        self.name = datetime.datetime.now().isoformat()
        self.left = []
        self.right = []
        self._prev_tm = 0

        self.next_state('exec_record')

    @state
    def exec_record(self, state_tm=0, initial_call=False):
        angle = self.angle_controller.get_angle()

        self.left.append([
            state_tm - self._prev_tm,
            self.drivetrain.get_left_encoder_meters(),
            self.drivetrain.get_left_encoder_velocity_meters(),
            angle
        ])
        self.right.append([
            state_tm - self._prev_tm,
            self.drivetrain.get_right_encoder_meters(),
            self.drivetrain.get_right_encoder_velocity_meters(),
            angle
        ])

        self._prev_tm = state_tm

    def done(self):
        if not self.is_recording:
            return

        print('[PathRecorderController] Recording finished - saving to %s' %
              self.name)

        traj = []
        for side in [self.left, self.right]:
            traj.append(
                [pf.Segment(dt=d[0], x=0, y=0, position=d[1], velocity=d[2],
                            acceleration=0, jerk=0, heading=d[3])
                 for d in self.left])

        pf.serialize_csv(path.join(path.dirname(__file__),
                                   '../recorded_paths', self.name + '-l.csv'),
                         traj[0])
        pf.serialize_csv(path.join(path.dirname(__file__),
                                   '../recorded_paths', self.name + '-r.csv'),
                         traj[1])

        pickle.dump(traj, open(path.join(path.dirname(__file__),
                                         '../recorded_paths',
                                         self.name + '.pickle'),
                               'wb'))

        self.is_recording = False
