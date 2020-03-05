import numpy as np


class LaneController:
    def __init__(self, parameters):
        self.parameters = parameters
        self.d_I = 0
        self.phi_I = 0
        self.prev_d_err = 0
        self.prev_phi_err = 0

    def update_parameters(self, parameters):
        """Updates parameters of LaneController object.

            Args:
                parameters (:obj:`dict`): dictionary containing the new parameters for LaneController object.
        """
        self.parameters = parameters

    def compute_control_action(self, d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance):
        """Main function, computes the control action given the current error signals.

        Given an estimate of the error, computes a control action (tuple of linear and angular velocity). This is done
        via a basic PI(D) controller with anti-reset windup logic.

        Args:
            d_err (:obj:`float`): error in meters in the lateral direction
            phi_err (:obj:`float`): error in radians in the heading direction
            dt (:obj:`float`): time since last command update
            wheels_cmd_exec (:obj:`bool`): confirmation that the wheel commands have been executed (to avoid
                                           integration while the robot does not move)
            stop_line_distance (:obj:`float`):  distance of the stop line, None if not detected.
        Returns:
            v (:obj:`float`): requested linear velocity in meters/second
            omega (:obj:`float`): requested angular velocity in radians/second
        """

        if dt is not None:
            self.integrate_errors(d_err, phi_err, dt)

        self.d_I = self.adjust_integral(d_err, self.d_I, self.parameters['~integral_bounds']['d'],
                                        self.parameters['~d_resolution'])
        self.phi_I = self.adjust_integral(phi_err, self.phi_I, self.parameters['~integral_bounds']['phi'],
                                          self.parameters['~phi_resolution'])

        self.reset_if_needed(d_err, phi_err, wheels_cmd_exec)

        # Scale the parameters linear such that their real value is at 0.22m/s
        omega = self.parameters['~k_d'] * (0.22 / self.parameters['~v_bar']) * d_err + \
            self.parameters['~k_theta'] * (0.22 / self.parameters['~v_bar']) * phi_err

        # apply magic conversion factors
        omega = omega * self.parameters['~omega_to_rad_per_s']

        self.prev_d_err = d_err
        self.prev_phi_err = phi_err

        v = self.compute_velocity(stop_line_distance)

        return v, omega

    def compute_velocity(self, stop_line_distance):
        """Linearly decrease velocity if approaching a stop line.

        If a stop line is detected, the velocity is linearly decreased to achieve a better stopping position,
        otherwise the nominal velocity is returned.

        Args:
            stop_line_distance (:obj:`float`): distance of the stop line, None if not detected.
        """
        if stop_line_distance is None:
            return self.parameters['~v_bar']
        else:
            # 60cm -> v_bar, 15cm -> v_bar/2
            d1, d2 = 0.8, 0.25
            a = self.parameters['~v_bar'] / (2 * (d1 - d2))
            b = self.parameters['~v_bar'] - a * d1
            v_new = a * stop_line_distance + b
            v = np.max([self.parameters['~v_bar'] / 2.0, np.min([self.parameters['~v_bar'], v_new])])
            return v

    def integrate_errors(self, d_err, phi_err, dt):
        """Integrates error signals in lateral and heading direction.
        Args:
            d_err (:obj:`float`): error in meters in the lateral direction
            phi_err (:obj:`float`): error in radians in the heading direction
            dt (:obj:`float`): time delay in seconds
        """
        self.d_I += d_err * dt
        self.phi_I += phi_err * dt

    def reset_if_needed(self, d_err, phi_err, wheels_cmd_exec):
        """Resets the integral error if needed.

        Resets the integral errors in `d` and `phi` if either the error sign changes, or if the robot is completely
        stopped (i.e. intersections).

        Args:
            d_err (:obj:`float`): error in meters in the lateral direction
            phi_err (:obj:`float`): error in radians in the heading direction
            wheels_cmd_exec (:obj:`bool`): confirmation that the wheel commands have been executed (to avoid
                                           integration while the robot does not move)
        """
        if np.sign(d_err) != np.sign(self.prev_d_err):
            self.d_I = 0
        if np.sign(phi_err) != np.sign(self.prev_phi_err):
            self.phi_I = 0
        if wheels_cmd_exec[0] == 0 and wheels_cmd_exec[1] == 0:
            self.d_I = 0
            self.phi_I = 0

    @staticmethod
    def adjust_integral(error, integral, bounds, resolution):
        """Bounds the integral error to avoid windup.

        Adjusts the integral error to remain in defined bounds, and cancels it if the error is smaller than the
        resolution of the error estimation.

        Args:
            error (:obj:`float`): current error value
            integral (:obj:`float`): current integral value
            bounds (:obj:`dict`): contains minimum and maximum value for the integral
            resolution (:obj:`float`): resolution of the error estimate

        Returns:
            integral (:obj:`float`): adjusted integral value
        """
        if integral > bounds['top']:
            integral = bounds['top']
        elif integral < bounds['bot']:
            integral = bounds['bot']
        elif abs(error) < resolution:
            integral = 0
        return integral
