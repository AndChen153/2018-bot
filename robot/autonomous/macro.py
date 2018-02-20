from robotpy_ext.autonomous import StatefulAutonomous, state

from controllers.macro_controller import MacroController


class Macro(StatefulAutonomous):

    MODE_NAME = 'Play Macro'
    DEFAULT = False

    aaa_macro_controller = MacroController

    @state(first=True)
    def play(self):
        self.aaa_macro_controller.play()
