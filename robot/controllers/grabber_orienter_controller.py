from enum import IntEnum

from magicbot import StateMachine, state, timed_state
from components import grabber


class GrabberOrienterSide(IntEnum):
    LEFT = 0
    RIGHT = 1
    FLIPPY = 2


class GrabberOrienterController(StateMachine):
    grabber = grabber.Grabber
    side = None

    def orient(self, side):
        self.side = side
        self.engage()

    @state(first=True)
    def first(self):
        if self.side == GrabberOrienterSide.LEFT:
            self.grabber.control_independently(0, 0.4)
        elif self.side == GrabberOrienterSide.RIGHT:
            self.grabber.control_independently(0.4, 0)
        elif self.side == GrabberOrienterSide.FLIPPY:
            self.next_state_now('flippy_left')

    @timed_state(duration=0.25, next_state='flippy_right')
    def flippy_left(self):
        self.grabber.control_independently(0, 0.4, priority=-1)

    @timed_state(duration=0.25, next_state='flippy_left')
    def flippy_right(self):
        self.grabber.control_independently(0.4, 0, priority=-1)
