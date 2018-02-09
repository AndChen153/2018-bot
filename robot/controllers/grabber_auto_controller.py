from components import targeting
from controllers import grabber_orienter_controller

# Only auto-grab when the cube is outside the bot; stop grabbing
# once we have the cube captured.
Y_CUTOFF = 0.2


class GrabberAutoController:
    grabber_orienter_controller = grabber_orienter_controller. \
        GrabberOrienterController
    targeting = targeting.Targeting

    def execute(self):
        data = self.targeting.get_data()
        if data.found and data.y >= Y_CUTOFF:
            self.grabber_orienter_controller.orient(
                grabber_orienter_controller.GrabberOrienterSide.FLIPPY)
