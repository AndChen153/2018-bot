from pyfrc.physics import drivetrains
from components.drivetrain import UNITS_PER_REV, DISTANCE_PER_REV


class PhysicsEngine(object):

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')

        self.drivetrain = drivetrains.TwoMotorDrivetrain(
            speed=12, x_wheelbase=2.5)

        # Don't worry about it
        self.kEncoder = 14000

        self.l_distance = 0
        self.r_distance = 0

    def update_sim(self, hal_data, now, tm_diff):
        try:
            l_motor = hal_data['CAN'][4]['value']
            r_motor = hal_data['CAN'][5]['value']

            speed, rotation = self.drivetrain.get_vector(l_motor, r_motor)

            self.physics_controller.drive(speed, rotation, tm_diff)

            # print('[physics] L: %s; R: %s; speed: %s, rot: %s' % \
            # (l_motor, r_motor, speed, rotation))

            # Handle reset quad encoders
            if self.l_distance and hal_data['CAN'][4]['quad_position'] == 0:
                self.l_distance = 0
            if self.r_distance and hal_data['CAN'][5]['quad_position'] == 0:
                self.r_distance = 0

            # Update encoders
            self.l_distance -= self.drivetrain.l_speed * tm_diff
            self.r_distance += self.drivetrain.r_speed * tm_diff

            hal_data['CAN'][4]['quad_position'] = int(
                self.l_distance * self.kEncoder)
            hal_data['CAN'][4]['quad_velocity'] = self.drivetrain.l_speed * \
                self.kEncoder

            hal_data['CAN'][5]['quad_position'] = int(
                self.r_distance * self.kEncoder)
            hal_data['CAN'][5]['quad_velocity'] = self.drivetrain.r_speed * \
                self.kEncoder

        except KeyError:
            # CAN not initialized
            pass
