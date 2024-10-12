from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range

from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter

from .state_estimator_abs import StateEstimatorAbs


class StateEstimatorUKF2D(StateEstimatorAbs):
    """
    Class that estimates the state of the drone using an Unscented Kalman Filter
    (UKF) applied to raw sensor data. The filter only tracks the quadcopter's
    motion along one spatial dimension: the global frame z-axis. In this
    simplified case, we assume that the drone's body frame orientation is
    aligned with the world frame (i.e., no roll, pitch, or yaw), and that it is
    only offset along the z-axis. It is called a 2D UKF because we track two
    state variables in the state vector: z position and z velocity.
    """

    def __init__(self):
        # Call the parent constructor
        super().__init__()

        # Initialize the estimator
        self.initialize_estimator()

        # Initialize flags and variables
        self.ready_to_filter = False
        self.printed_filter_start_notice = False
        self.got_altitude = False
        self.got_imu = False
        self.in_callback = False

        # Time management
        self.last_state_transition_time = None
        self.dt = None

        # Last control input
        self.last_control_input = np.array([0.0])

    def initialize_estimator(self):
        """
        Initialize the parameters of the Unscented Kalman Filter (UKF) that is
        used to estimate the state of the drone.
        """
        # Number of state variables being tracked
        self.state_vector_dim = 2

        # Number of measurement variables that the drone receives
        self.measurement_vector_dim = 1

        # Function to generate sigma points for the UKF
        sigma_points = MerweScaledSigmaPoints(
            n=self.state_vector_dim,
            alpha=0.1,
            beta=2.0,
            kappa=(3.0 - self.state_vector_dim),
        )

        # Create the UKF object
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.state_vector_dim,
            dim_z=self.measurement_vector_dim,
            dt=1.0,
            hx=self.measurement_function,
            fx=self.state_transition_function,
            points=sigma_points,
        )
        self.initialize_ukf_matrices()

    def initialize_ukf_matrices(self):
        """
        Initialize the covariance matrices of the UKF.
        """
        # Initialize state covariance matrix P
        self.ukf.P = np.diag([0.1, 0.2])

        # Initialize the process noise covariance matrix Q
        self.ukf.Q = np.diag([0.01, 1.0]) * 0.005

        # Initialize the measurement covariance matrix R
        self.measurement_cov_altitude = np.array([2.2221e-05])

    def process_imu(self, imu_data: Imu):
        """
        Process IMU data and perform prediction step.
        """
        if self.in_callback:
            return
        self.in_callback = True

        self.last_control_input = np.array([imu_data.linear_acceleration.z])

        # Adjust process noise based on acceleration magnitude
        if abs(imu_data.linear_acceleration.z) < 0.3:
            # Lower process noise if acceleration is low
            self.ukf.Q = np.diag([0.01, 1.0]) * 0.0005
        else:
            self.ukf.Q = np.diag([0.01, 1.0]) * 0.005

        if self.ready_to_filter:
            self.update_input_time(imu_data)
            self.compute_prior()
            self.update_current_state()
        else:
            self.initialize_input_time(imu_data)
            self.got_imu = True
            self.check_if_ready_to_filter()

        self.in_callback = False

    def process_range(self, range_data: Range):
        """
        Process altitude data and perform prediction and update steps.
        """
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
            self.ukf.x[0] = range_data.range  # z position
            self.ukf.x[1] = 0.0  # z velocity initialized to zero
            self.ukf.P[0, 0] = self.measurement_cov_altitude[0]
            self.got_altitude = True
            self.check_if_ready_to_filter()

        self.in_callback = False

    def process_pose(self, pose_data: PoseStamped):
        return super().process_pose(pose_data)

    def process_twist(self, twist_data: TwistStamped):
        return super().process_twist(twist_data)

    def compute_prior(self):
        """
        Compute the prior for the UKF based on the current state, control input, and time step.
        """
        self.ukf.predict(dt=self.dt, u=self.last_control_input)

    def update_current_state(self):
        """
        Publish the current state estimate and covariance from the UKF.
        """
        state_msg = Odometry()
        state_msg.header.stamp.secs = self.last_time_secs
        state_msg.header.stamp.nsecs = self.last_time_nsecs
        state_msg.header.frame_id = "global"

        # Get the current state estimate from self.ukf.x
        state_msg.pose.pose.position.z = self.ukf.x[0]
        state_msg.twist.twist.linear.z = self.ukf.x[1]

        # Fill the rest of the message with NaN
        state_msg.pose.pose.position.x = np.nan
        state_msg.pose.pose.position.y = np.nan
        state_msg.pose.pose.orientation.x = np.nan
        state_msg.pose.pose.orientation.y = np.nan
        state_msg.pose.pose.orientation.z = np.nan
        state_msg.pose.pose.orientation.w = np.nan
        state_msg.twist.twist.linear.x = np.nan
        state_msg.twist.twist.linear.y = np.nan
        state_msg.twist.twist.angular.x = np.nan
        state_msg.twist.twist.angular.y = np.nan
        state_msg.twist.twist.angular.z = np.nan

        # Prepare covariance matrices
        pose_cov_mat = np.full((36,), np.nan)
        twist_cov_mat = np.full((36,), np.nan)
        pose_cov_mat[14] = self.ukf.P[0, 0]  # z variance
        twist_cov_mat[14] = self.ukf.P[1, 1]  # z velocity variance

        # Add covariances to message
        state_msg.pose.covariance = pose_cov_mat.tolist()
        state_msg.twist.covariance = twist_cov_mat.tolist()

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
        if self.dt < 0:
            self.dt = 0
        self.last_state_transition_time = new_time

    def check_if_ready_to_filter(self):
        """
        Check if the estimator is ready to start filtering.
        """
        self.ready_to_filter = self.got_altitude and self.got_imu

    def state_transition_function(self, x, dt, u):
        """
        The state transition function to compute the prior in the prediction step,
        propagating the state to the next time step.
        """
        # State transition matrix F
        F = np.array([[1, dt], [0, 1]])

        # Change from control input (acceleration)
        change_from_control_input = np.array([0, u[0] * dt])

        x_output = np.dot(F, x) + change_from_control_input
        return x_output

    def measurement_function(self, x):
        """
        Default measurement function (not used in this implementation).
        """
        pass

    def measurement_function_altitude(self, x):
        """
        Measurement function for altitude data.
        """
        return np.array([x[0]])
