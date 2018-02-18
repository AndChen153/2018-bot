from magicbot import StateMachine, state

from controllers.position_controller import PositionController


class HoldPositionController(StateMachine):

    position_controller = PositionController

    @state(first=True)
    def prepare_to_hold(self):
        self.position_controller.reset_position_and_heading()
        self.next_state('hold_position')

    @state
    def hold_position(self):
        self.position_controller.move_to(0)
