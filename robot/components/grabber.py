from magicbot import tunable
from ctre import WPI_TalonSRX
from enum import IntEnum

from components.bot import Bot, LedState

CUBE_CURRENT_CUTOFF = 10


class GrabberState(IntEnum):
    DISABLED = 0
    INTAKING = 1
    DEPOSITING = 2


class Grabber:

    intake_speed = tunable(0.8)
    deposit_speed = tunable(0.5)
    current_limit = tunable(0.0)

    left_motor = WPI_TalonSRX
    right_motor = WPI_TalonSRX

    bot = Bot

    def setup(self):
        self.pending_state = GrabberState.DISABLED
        self.pending_independent_control = None
        self.independent_control_priority = None

        self._has_cube = False

        self.right_motor.setInverted(True)

    def has_cube_intake_current(self):
        try:
            return self.left_motor.getOutputCurrent() > CUBE_CURRENT_CUTOFF
        except NotImplementedError:
            return False

    def intake(self):
        self.pending_state = GrabberState.INTAKING

    def deposit(self):
        self.pending_state = GrabberState.DEPOSITING

    def control_independently(self, left, right, priority=None):
        '''
        Independently control motors. If priority=-1, will be superseded
        by any other intake/deposit commands.
        '''
        self.pending_independent_control = [left, right]
        self.independent_control_priority = priority

    def execute(self):
        # Independent motor control - bypass all the normal controls
        if self.pending_independent_control:

            # If low priority independent control and there's another command
            # that supersedes, give up on the independent control and allow
            # regular execution to continue.
            if self.independent_control_priority == -1 and \
                    self.pending_state != GrabberState.DISABLED:
                self.pending_independent_control = None

            # Otherwise, run the indepdent control and then abort early.
            else:
                self.left_motor.set(self.pending_independent_control[0])
                self.right_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                     self.pending_independent_control[1])
                self.pending_independent_control = None
                return

        self.right_motor.set(WPI_TalonSRX.ControlMode.Follower,
                             self.left_motor.getDeviceID())

        if self.pending_state == GrabberState.DISABLED:
            self.left_motor.set(0)
        elif self.pending_state == GrabberState.INTAKING:
            self.left_motor.set(self.intake_speed)
            self._has_cube = self.has_cube_intake_current()
        elif self.pending_state == GrabberState.DEPOSITING:
            self.left_motor.set(-self.deposit_speed)
            self._has_cube = False

        self.pending_state = GrabberState.DISABLED

        # Led update
        self.bot.set_led_state(
            LedState.GREEN if self._has_cube else LedState.RED)

    def on_disabled(self):
        self.left_motor.set(0)

    def get_state(self):
        return {
            'pending_state': self.pending_state
        }

    def put_state(self, state):
        self.pending_state = state['pending_state']
