from components import targeting, elevator
from controllers import grabber_orienter_controller

# Only auto-grab when the cube is outside the bot; stop grabbing
# once we have the cube captured.
Y_CUTOFF = 0.5


class GrabberAutoController:
    grabber_orienter_controller = grabber_orienter_controller. \
        GrabberOrienterController
    targeting = targeting.Targeting
    elevator = elevator.Elevator

    def setup(self):
        self.disabled = False

    def disable(self):
        self.disabled = True

    def enable(self):
        self.disabled = False

    def execute(self):
        if self.disabled:
            return

        data = self.targeting.get_data()
        if self.elevator.is_at_ground() and data.found and data.y >= Y_CUTOFF:
            self.grabber_orienter_controller.orient(
                grabber_orienter_controller.GrabberOrienterSide.FLIPPY)
