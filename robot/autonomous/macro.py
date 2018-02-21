from robotpy_ext.autonomous import StatefulAutonomous, state

from controllers.macro_controller import MacroController
from components.field import Field, SwitchState


class Macro(StatefulAutonomous):

    aaa_macro_controller = MacroController
    field = Field

    MACRO_NAME = None
    MACRO_NAME_LEFT = None
    MACRO_NAME_RIGHT = None

    @state(first=True)
    def start(self):
        if self.MACRO_NAME:
            self._macro_name = self.MACRO_NAME
            self.next_state('play')
        else:
            switch_side = self.field.get_switch_side()
            if switch_side is not None:
                self._macro_name = self.MACRO_NAME_RIGHT if \
                    switch_side == SwitchState.RIGHT else \
                    self.MACRO_NAME_LEFT
                self.next_state('play')

    @state
    def play(self):
        self.aaa_macro_controller.play(self._macro_name)


class TwoCube(Macro):

    MODE_NAME = '[Macro] Two Cube From Center'
    MACRO_NAME_LEFT = 'two_cube_center_leftswitch'
    MACRO_NAME_RIGHT = 'two_cube_center_rightswitch'
    DEFAULT = False


class TwoCubeLeftSide(Macro):

    MODE_NAME = '[Macro] Two Cube From Left Side'
    MACRO_NAME_LEFT = 'two_cube_left_leftswitch'
    MACRO_NAME_RIGHT = 'two_cube_left_rightswitch'
    DEFAULT = False


class TwoCubeRightSide(Macro):

    MODE_NAME = '[Macro] Two Cube From Right Side'
    MACRO_NAME_LEFT = 'two_cube_right_leftswitch'
    MACRO_NAME_RIGHT = 'two_cube_right_rightswitch'
    DEFAULT = False


class Test(Macro):

    MODE_NAME = '[Macro] Test Shite'
    MACRO_NAME_LEFT = 'idk'
    MACRO_NAME_RIGHT = 'idk'
    DEFAULT = False
