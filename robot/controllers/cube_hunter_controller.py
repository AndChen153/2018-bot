from magicbot import state, StateMachine, tunable

from common.util import abs_clamp

from components.drivetrain import Drivetrain
from components.targeting import Targeting


class CubeHunterController(StateMachine):
    drivetrain = Drivetrain
    targeting = Targeting

    ideal_x = tunable(0)
    ideal_y = tunable(0)

    x_tolerance = tunable(0.1)
    y_tolerance = tunable(0.1)

    y_factor = tunable(0.001)
    rotation_factor = tunable(0.5)
    min_rotation = tunable(0)
    max_rotation = tunable(1)
    high_speed_rotation_cutoff = tunable(0.5)
    high_speed_rotation_speed = tunable(0.4)

    min_speed = tunable(0.1)
    max_speed = tunable(0.3)

    def seek(self):
        self.engage()

    def is_seeking(self):
        return self.is_executing

    def _move_to_position(self):
        data = self.targeting.get_data()
        speed = 0
        rotation = 0
        min_speed = self.min_speed

        print('data', data)

        # Just give up if we can't find the cube :'(
        if not data.found:
            return False

        x_offset = data.x - self.ideal_x
        y_offset = data.y - self.ideal_y

        within_x_tolerance = abs(data.x - self.ideal_x) <= self.x_tolerance
        within_y_tolerance = abs(data.y - self.ideal_y) <= self.y_tolerance

        if not within_x_tolerance:
            rotation = x_offset * self.rotation_factor
            if x_offset > self.high_speed_rotation_cutoff:
                min_speed = self.high_speed_rotation_speed

        if not within_y_tolerance:
            speed = y_offset * self.y_factor

        speed = abs_clamp(speed, min_speed, self.max_speed)
        rotation = abs_clamp(rotation, self.min_rotation, self.max_rotation)

        # Rotation: flip based upon forwards/backwards
        rotation *= abs(speed) / speed

        self.drivetrain.differential_drive(speed, rotation, squared=False)

        print('cube_hunter#moving_to_position', 'speed', speed, 'rotation', rotation)

        return within_x_tolerance and within_y_tolerance

    @state(first=True)
    def inital_state(self):
        self.next_state_now('moving_to_position')

    @state
    def moving_to_position(self):
        if self._move_to_position():
            self.next_state('done')

    def done(self):
        super().done()

    def on_disable(self):
        self.done()
