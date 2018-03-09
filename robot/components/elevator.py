import math
from magicbot import tunable
from ctre import WPI_TalonSRX
from enum import IntEnum
from wpilib import DoubleSolenoid, DigitalInput

from constants import TALON_TIMEOUT

# 12T #35 sprocket diameter: https://www.vexrobotics.com/35-sprockets.html
UNITS_PER_REV = 4096
DISTANCE_PER_REV = math.pi * 1.786  # pi * sprocket diameter

GROUND_CUTOFF = 250
POSITION_TOLERANCE = 250


class ElevatorState(IntEnum):
    LOCKED = 0
    RELEASED = 1


class ElevatorPosition(IntEnum):
    GROUND = 0
    CARRYING = 2000
    SWITCH = 23000


class Elevator:

    USE_MOTIONMAGIC = False

    motor = WPI_TalonSRX
    solenoid = DoubleSolenoid
    reverse_limit = DigitalInput

    kFreeSpeed = tunable(1)
    kZeroingSpeed = tunable(0.3)
    kP = tunable(0.5)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)

    kCruiseVelocity = 15000
    kAcceleration = 6000

    setpoint = tunable(0)
    value = tunable(0)
    error = tunable(0)

    def setup(self):
        self.state = ElevatorState.LOCKED
        self.pending_position = None
        self.pending_drive = None

        self.has_zeroed = False
        self.needs_brake = False
        self.braking_direction = None

        self.motor.setInverted(True)
        self.motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.motor.selectProfileSlot(0, 0)
        self.motor.setSensorPhase(True)

        self.motor.config_kP(0, self.kP, 0)
        self.motor.config_kI(0, self.kI, 0)
        self.motor.config_kD(0, self.kD, 0)
        self.motor.config_kF(0, self.kF, 0)

        try:
            self.motor.configMotionCruiseVelocity(self.kCruiseVelocity, 0)
            self.motor.configMotionAcceleration(self.kAcceleration, 0)
        except NotImplementedError:
            # Simulator - no motion profiling support
            self.USE_MOTIONMAGIC = False

    def is_encoder_connected(self):
        return self.motor.getPulseWidthRiseToRiseUs() != 0

    def get_encoder_position(self):
        return self.motor.getSelectedSensorPosition(0)

    def is_at_ground(self):
        return self.get_encoder_position() <= GROUND_CUTOFF

    def is_at_position(self, position):
        return abs(self.get_encoder_position() - position) <= \
            POSITION_TOLERANCE

    def lock(self):
        self.state = ElevatorState.LOCKED

    def release_lock(self):
        self.state = ElevatorState.RELEASED

    def raise_to_switch(self):
        self.pending_position = ElevatorPosition.SWITCH

    def raise_to_carry(self):
        self.pending_position = ElevatorPosition.CARRYING

    def lower_to_ground(self):
        self.pending_position = ElevatorPosition.GROUND

    def toggle_carry_ground(self):
        if self.pending_position == ElevatorPosition.GROUND:
            self.pending_position = ElevatorPosition.CARRYING
        else:
            self.pending_position = ElevatorPosition.GROUND

    def move_incremental(self, amount):
        '''
        Move `amount` inches.
        '''
        self.pending_position = self.get_encoder_position() + \
            amount / DISTANCE_PER_REV * UNITS_PER_REV

    def raise_freely(self):
        self.pending_drive = self.kFreeSpeed

    def lower_freely(self):
        self.pending_drive = -self.kFreeSpeed

    def execute(self):
        # For debugging
        # print('elevator', 'drive', self.pending_drive,
        #       'lim', self.reverse_limit.get(),
        #       'pos', self.pending_position,
        #       'setpoint', self.setpoint,
        #       'val', self.value,
        #       'err', self.error)

        # Brake - apply the brake either when we reach peak of movement
        # (for upwards motion), and thus ds/dt = v = 0, or else immediately
        # if we're traveling downwards (since no e.z. way to sense gravity vs
        # intertial movement).
        if self.needs_brake:
            velocity = self.motor.getQuadratureVelocity()
            if velocity == 0 or \
                    self.braking_direction == -1 or \
                    velocity / abs(velocity) != self.braking_direction:
                self.pending_position = self.motor.getQuadraturePosition()
                self.needs_brake = False
                self.braking_direction = None

        # Zero the encoder when hits bottom of elevator - limit switch NC
        # Also ensure we don't drive past the bottom limit.
        if not self.reverse_limit.get():
            self.motor.setQuadraturePosition(0, TALON_TIMEOUT)
            self.has_zeroed = True
            if self.pending_drive and self.pending_drive <= 0:
                self.pending_drive = None

        # Elevator motor
        if self.pending_drive:
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                           self.pending_drive)
            self.pending_drive = None
            self.pending_position = None  # Clear old pending position

        elif self.pending_position is not None and self.is_encoder_connected():
            # Note: we don't clear the pending position so that we keep
            # on driving to the position in subsequent execute() cycles.
            if not self.has_zeroed and \
                    self.pending_position == ElevatorPosition.GROUND:
                # Drive downwards until we zero it... and cross our fingers...
                self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                               -self.kZeroingSpeed)
            elif self.has_zeroed:  # Don't drive positionally if not zeroed
                if self.USE_MOTIONMAGIC:
                    self.motor.set(WPI_TalonSRX.ControlMode.MotionMagic,
                                   self.pending_position)
                else:
                    self.motor.set(WPI_TalonSRX.ControlMode.Position,
                                   self.pending_position)
        else:
            if self.is_encoder_connected():
                # If no command, hold position in place (basically, a more
                # "aggressive" brake mode to prevent any slippage).
                self.needs_brake = True
                velocity = self.motor.getQuadratureVelocity()
                self.braking_direction = velocity / abs(velocity or 1)
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)

        # Elevator deployment/retraction
        if self.state == ElevatorState.LOCKED:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.state == ElevatorState.RELEASED:
            self.solenoid.set(DoubleSolenoid.Value.kForward)

        # Update dashboard PID values
        if self.pending_position:
            try:
                self.setpoint = self.motor.getClosedLoopTarget(0)
                self.value = self.motor.getSelectedSensorPosition(0)
                self.error = self.motor.getClosedLoopError(0)
            except NotImplementedError:
                # Simulator doesn't implement getError
                pass

    def get_state(self):
        return {
            'pending_position': self.pending_position
        }

    def put_state(self, state):
        self.pending_position = state['pending_position']
