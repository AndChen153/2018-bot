from pyfrc.physics import drivetrains


class PhysicsEngine(object):

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')

    def update_sim(self, hal_data, now, tm_diff):
        try:
            # Left motor
            l_speed = int(
                20000 * hal_data['CAN'][4]['value'] * tm_diff)
            hal_data['CAN'][4]['quad_position'] += l_speed
            hal_data['CAN'][4]['quad_velocity'] = l_speed

            # Right motor
            r_speed = int(
                20000 * hal_data['CAN'][5]['value'] * tm_diff)
            hal_data['CAN'][5]['quad_position'] += r_speed
            hal_data['CAN'][5]['quad_velocity'] = r_speed

            l_motor = hal_data['CAN'][4]['value']
            r_motor = hal_data['CAN'][5]['value']

            speed, rotation = drivetrains.two_motor_drivetrain(
                l_motor, r_motor, x_wheelbase=2.5, speed=3.66)
            self.physics_controller.drive(speed, rotation, tm_diff)

        except KeyError:
            # CAN not initialized
            pass
