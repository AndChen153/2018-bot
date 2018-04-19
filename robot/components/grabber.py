from magicbot import tunable
from ctre import WPI_TalonSRX
from enum import IntEnum

from components.bot import Bot, LedState
from components.elevator import Elevator

CUBE_CURRENT_CUTOFF = 10

SHOULD_HOLD_POSITION = False


class GrabberState(IntEnum):
    DISABLED = 0
    INTAKING = 1
    DEPOSITING = 2


class Grabber:

    intake_speed = tunable(1.0)  # originally: 0.8
    deposit_speed = tunable(0.45)
    current_limit = tunable(0.0)

    kP = 2
    kI = 0
    kD = 0
    kF = 0

    elevator = Elevator

    left_motor = WPI_TalonSRX
    right_motor = WPI_TalonSRX

    bot = Bot

    def setup(self):
        self.pending_state = GrabberState.DISABLED
        self.pending_independent_control = None
        self.independent_control_priority = None

        self._has_cube = False
        self._has_braked = False
        self._brake_pos = (0, 0)

        self.left_motor.setInverted(True)
        self.right_motor.setInverted(False)
        self.left_motor.setSensorPhase(True)
        self.right_motor.setSensorPhase(True)

        for motor in [self.left_motor, self.right_motor]:
            motor.config_kP(0, self.kP, 0)
            motor.config_kI(0, self.kI, 0)
            motor.config_kD(0, self.kD, 0)
            motor.config_kF(0, self.kF, 0)
            motor.configSelectedFeedbackSensor(
                WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)

    def are_encoders_connected(self):
        return self.left_motor.getPulseWidthRiseToRiseUs() != 0 and \
            self.right_motor.getPulseWidthRiseToRiseUs() != 0

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
                self.left_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                    self.pending_independent_control[0])
                self.right_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                     self.pending_independent_control[1])
                self.pending_independent_control = None
                self._has_braked = False
                self.pending_state = GrabberState.DISABLED
                return

        if self.pending_state == GrabberState.DISABLED:
            if self.elevator.is_at_ground() or not SHOULD_HOLD_POSITION:
                self.left_motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)
                self.right_motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)
                self._has_braked = False
                return

            if not self._has_braked:
                print('[grabber] set new brake pos')
                self._brake_pos = (self.left_motor.getQuadraturePosition(),
                                   self.right_motor.getQuadraturePosition())
                self._has_braked = True

            print('[grabber] braking to ', self._brake_pos)
            print('[grabber] curr ',
                  self.left_motor.getQuadraturePosition(),
                  self.right_motor.getQuadraturePosition())

            # self.left_motor.set(WPI_TalonSRX.ControlMode.Velocity, 0)
            # self.right_motor.set(WPI_TalonSRX.ControlMode.Velocity, 0)
            if self.left_motor.getPulseWidthRiseToRiseUs() != 0:
                self.left_motor.set(WPI_TalonSRX.ControlMode.Position,
                                    self._brake_pos[0])
            else:
                self.left_motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)

            if self.right_motor.getPulseWidthRiseToRiseUs() != 0:
                self.right_motor.set(WPI_TalonSRX.ControlMode.Position,
                                     self._brake_pos[1])
            else:
                self.right_motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)

            return

        self.right_motor.set(WPI_TalonSRX.ControlMode.Follower,
                             self.left_motor.getDeviceID())

        if self.pending_state == GrabberState.INTAKING:
            self.left_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                self.intake_speed)
            self._has_braked = False
            self._has_cube = self.has_cube_intake_current()
        elif self.pending_state == GrabberState.DEPOSITING:
            self.left_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                -self.deposit_speed)
            self._has_cube = False
            self._has_braked = False

        self.pending_state = GrabberState.DISABLED

        # Led update
        self.bot.set_led_state(
            LedState.GREEN if self._has_cube else LedState.RED)

    def on_disabled(self):
        self.left_motor.set(0)
        self.right_motor.set(0)
        self._has_braked = False

    def get_state(self):
        return {
            'pending_state': self.pending_state
        }

    def put_state(self, state):
        self.pending_state = state['pending_state']
