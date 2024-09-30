#!/usr/bin/env python3

import tf

import numpy as np
import tf.transformations
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import pi

from .state_estimator_abs import StateEstimatorAbs


class StateEstimatorUKF7D(StateEstimatorAbs):
    """
    UKF-based state estimator for a 7D state space using various sensor inputs.
    """

    def __init__(
        self,
        altitude_throttled=False,
        imu_throttled=False,
        optical_flow_throttled=False,
        camera_pose_throttled=False,
    ):
        # Call the parent constructor
        super().__init__()

        # Initialize the estimator
        self.initialize_estimator()

        # Initialize flags and variables
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_imu = False
        self.got_altitude = False
        self.got_optical_flow = False
        self.got_camera_pose = False
        self.in_callback = False

        # Time management
        self.last_state_transition_time = None
        self.dt = None

        # Last control input
        self.last_control_input = np.array([0.0, 0.0, 0.0])


    def initialize_estimator(self):
        """Initialize the UKF estimator parameters."""
        self.state_vector_dim = 7
        self.measurement_vector_dim = 6

        sigma_points = MerweScaledSigmaPoints(
            n=self.state_vector_dim,
            alpha=0.1,
            beta=2.0,
            kappa=(3.0 - self.state_vector_dim),
        )

        self.ukf = UnscentedKalmanFilter(
            dim_x=self.state_vector_dim,
            dim_z=self.measurement_vector_dim,
            dt=1.0,
            hx=self.measurement_function,
            fx=self.state_transition_function,
            points=sigma_points,
            residual_x=self.residual_x_account_for_angles,
        )

        self.initialize_ukf_matrices()

    def initialize_ukf_matrices(self):
        """Initialize the covariance matrices for the UKF."""
        self.ukf.P = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.0005])
        self.ukf.Q = np.diag([0.01, 0.01, 0.01, 1.0, 1.0, 1.0, 0.1]) * 0.005

        # Measurement covariance matrices
        self.measurement_cov_altitude = np.array([2.2221e-05])
        self.measurement_cov_optical_flow = np.diag([0.01, 0.01])
        self.measurement_cov_camera_pose = np.diag([0.0025, 0.0025, 0.0003])

    def process_imu(self, imu_data: Imu):
        """Process IMU data and perform prediction and update steps."""
        if self.in_callback:
            return
        self.in_callback = True

        self.last_control_input = np.array(
            [
                imu_data.linear_acceleration.x,
                imu_data.linear_acceleration.y,
                imu_data.linear_acceleration.z,
            ]
        )
        # Assuming we might use angular velocities in future
        # self.angular_velocity = imu_data.angular_velocity

        if self.ready_to_filter:
            self.update_input_time(imu_data)
            self.compute_prior()

            # No measurement update from IMU in this simplified example
            self.update_current_state()
        else:
            self.initialize_input_time(imu_data)
            self.got_imu = True
            self.check_if_ready_to_filter()

        self.in_callback = False

    def process_range(self, range_data):
        """Process altitude data and perform prediction and update steps."""
        if self.in_callback:
            return
        self.in_callback = True

        if self.ready_to_filter:
            self.update_input_time(range_data)
            self.compute_prior()

            measurement_z = np.array([range_data.range])

            self.ukf.update(
                measurement_z,
                hx=self.measurement_function_altitude,
                R=self.measurement_cov_altitude,
            )
            self.update_current_state()
        else:
            self.initialize_input_time(range_data)
            self.ukf.x[2] = range_data.range  # Altitude (z position)
            self.ukf.P[2, 2] = self.measurement_cov_altitude[0]
            self.got_altitude = True
            self.check_if_ready_to_filter()

        self.in_callback = False

    def process_twist(self, twist_data: TwistStamped):
        """Process optical flow data and perform prediction and update steps."""
        if self.in_callback:
            return
        self.in_callback = True

        if self.ready_to_filter:
            self.update_input_time(twist_data)
            self.compute_prior()

            measurement_z = np.array(
                [twist_data.twist.linear.x, twist_data.twist.linear.y]
            )

            self.ukf.update(
                measurement_z,
                hx=self.measurement_function_optical_flow,
                R=self.measurement_cov_optical_flow,
            )
            self.update_current_state()
        else:
            self.initialize_input_time(twist_data)
            self.ukf.x[3] = twist_data.twist.linear.x  # x velocity
            self.ukf.x[4] = twist_data.twist.linear.y  # y velocity
            self.ukf.P[3, 3] = self.measurement_cov_optical_flow[0, 0]
            self.ukf.P[4, 4] = self.measurement_cov_optical_flow[1, 1]
            self.got_optical_flow = True
            self.check_if_ready_to_filter()

        self.in_callback = False

    def process_pose(self, pose_data):
        """Process camera pose data and perform prediction and update steps."""
        if self.in_callback:
            return
        self.in_callback = True

        _, _, yaw = tf.transformations.euler_from_quaternion(
            [
                pose_data.pose.orientation.x,
                pose_data.pose.orientation.y,
                pose_data.pose.orientation.z,
                pose_data.pose.orientation.w,
            ]
        )

        if self.ready_to_filter:
            self.update_input_time(pose_data)
            self.compute_prior()

            measurement_z = np.array(
                [
                    pose_data.pose.position.x,
                    pose_data.pose.position.y,
                    yaw,
                ]
            )

            self.ukf.update(
                measurement_z,
                hx=self.measurement_function_camera_pose,
                R=self.measurement_cov_camera_pose,
            )
            self.update_current_state()
        else:
            self.initialize_input_time(pose_data)
            self.ukf.x[0] = pose_data.pose.position.x
            self.ukf.x[1] = pose_data.pose.position.y
            self.ukf.x[5] = yaw
            self.ukf.P[0, 0] = self.measurement_cov_camera_pose[0, 0]
            self.ukf.P[1, 1] = self.measurement_cov_camera_pose[1, 1]
            self.ukf.P[5, 5] = self.measurement_cov_camera_pose[2, 2]
            self.got_camera_pose = True
            self.check_if_ready_to_filter()

        self.in_callback = False

    def compute_prior(self):
        """Predict the state using the UKF."""
        self.ukf.predict(dt=self.dt, u=self.last_control_input)

    def update_current_state(self):
        """Publish the current state estimate and covariance from the UKF."""
        state_msg = Odometry()
        state_msg.header.stamp.secs = self.last_time_secs
        state_msg.header.stamp.nsecs = self.last_time_nsecs
        state_msg.header.frame_id = "global"

        # Get the current state estimate from self.ukf.x
        state_msg.pose.pose.position.x = self.ukf.x[0]
        state_msg.pose.pose.position.y = self.ukf.x[1]
        state_msg.pose.pose.position.z = self.ukf.x[2]
        state_msg.twist.twist.linear.x = self.ukf.x[3]
        state_msg.twist.twist.linear.y = self.ukf.x[4]
        state_msg.twist.twist.linear.z = self.ukf.x[5]

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.ukf.x[5])
        state_msg.pose.pose.orientation.x = quaternion[0]
        state_msg.pose.pose.orientation.y = quaternion[1]
        state_msg.pose.pose.orientation.z = quaternion[2]
        state_msg.pose.pose.orientation.w = quaternion[3]

        # Extract covariances
        P = self.ukf.P
        # Pose covariance
        pose_covariance = np.zeros(36)
        pose_covariance[0] = P[0, 0]  # x
        pose_covariance[7] = P[1, 1]  # y
        pose_covariance[14] = P[2, 2]  # z
        pose_covariance[21] = 0.0  # Roll variance (not estimated)
        pose_covariance[28] = 0.0  # Pitch variance (not estimated)
        pose_covariance[35] = P[5, 5]  # Yaw

        state_msg.pose.covariance = pose_covariance.tolist()

        # Twist covariance
        twist_covariance = np.zeros(36)
        twist_covariance[0] = P[3, 3]  # x_vel
        twist_covariance[7] = P[4, 4]  # y_vel
        twist_covariance[14] = P[5, 5]  # z_vel

        state_msg.twist.covariance = twist_covariance.tolist()

        self.state = state_msg

    def initialize_input_time(self, msg):
        """
        Initialize the input time based on the timestamp in the header of a ROS message.
        """
        self.last_time_secs = msg.header.stamp.secs
        self.last_time_nsecs = msg.header.stamp.nsecs
        self.last_state_transition_time = self.last_time_secs + self.last_time_nsecs * 1e-9

    def update_input_time(self, msg):
        """
        Update the time at which we have received the most recent input, based on the timestamp.
        """
        self.last_time_secs = msg.header.stamp.secs
        self.last_time_nsecs = msg.header.stamp.nsecs
        new_time = self.last_time_secs + self.last_time_nsecs * 1e-9
        self.dt = new_time - self.last_state_transition_time
        self.last_state_transition_time = new_time

    def check_if_ready_to_filter(self):
        """Check if the estimator is ready to start filtering."""
        self.ready_to_filter = self.got_imu and self.got_altitude

    def state_transition_function(self, x, dt, u):
        """Define the state transition function for the UKF."""
        F = np.eye(self.state_vector_dim)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Compute the change from the control input (accelerations)
        accelerations = u
        velocities = accelerations * dt
        change_from_control_input = np.array(
            [0, 0, 0, velocities[0], velocities[1], velocities[2], 0]
        )
        x_output = np.dot(F, x) + change_from_control_input
        x_output[5] = x_output[5] % (2 * pi)  # Ensure yaw stays within [0, 2*pi]
        return x_output

    def residual_x_account_for_angles(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Compute the residual for the state vector x, accounting for angles."""
        output_residual = a - b
        # Adjust yaw angle residual
        yaw_residual = output_residual[5]
        yaw_residual = (yaw_residual + pi) % (2 * pi) - pi
        output_residual[5] = yaw_residual
        return output_residual

    def measurement_function(self, x):
        """Default measurement function (not used in this implementation)."""
        pass

    def measurement_function_altitude(self, x):
        """Measurement function for altitude data."""
        return np.array([x[2]])

    def measurement_function_optical_flow(self, x):
        """Measurement function for optical flow data."""
        return np.array([x[3], x[4]])

    def measurement_function_camera_pose(self, x):
        """Measurement function for camera pose data."""
        return np.array([x[0], x[1], x[5]])

    def measurement_function_complete(self, x):
        """Measurement function when all measurements are combined."""
        return np.array([x[2], x[0], x[1], x[3], x[4], x[5]])
