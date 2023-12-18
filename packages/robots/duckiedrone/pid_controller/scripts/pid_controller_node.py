#!/usr/bin/env python3

import signal

import rospy
import sys
import tf
from duckietown_msgs.msg import DroneControl as RC
from duckietown_msgs.msg import DroneMode as Mode
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool, Float32

from duckietown.dtros import DTROS, NodeType
from pid_class import PID, PIDaxis
from three_dim_vec import Position, Velocity, Error, RPY


class PIDController(DTROS):
    """
    Controls the flight of the drone by running a PID controller on the
    error calculated by the desired and current velocity and position of the drone
    """

    def __init__(self):
        super(PIDController, self).__init__(node_name="pid_controller_node",
                                            node_type=NodeType.CONTROL)
        # set max range for time of flight
        self.frequency = rospy.get_param("~frequency")
        self.max_height = rospy.get_param("~max_height")
        self.hover_height = rospy.get_param("~hover_height")

        # Initialize the current and desired modes (initialized both to DISARMED)
        self.previous_mode = 0
        self.current_mode = 0

        # Initialize in velocity control
        self.position_control = False
        self.last_position_control = False

        # Initialize the current and desired positions
        self.current_position = Position()
        # TODO: 0.5 is hovering height? hardcoded?
        self.desired_position = Position(z=0.5)
        self.last_desired_position = Position(z=0.5)

        # Initialize the position error
        self.position_error = Error()

        # Initialize the current and desired velocities
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        # Initialize the velocity error
        self.velocity_error = Error()

        # Set the distance that a velocity command will move the drone (m)
        self.desired_velocity_travel_distance = 0.1

        # Set a static duration that a velocity command will be held
        self.desired_velocity_travel_time = 0.1

        # Set a static duration that a yaw velocity command will be held
        self.desired_yaw_velocity_travel_time = 0.25

        # Store the start time of the desired velocities
        self.desired_velocity_start_time = None
        self.desired_yaw_velocity_start_time = None

        # Initialize the primary PID
        self.pid = PID()

        # Initialize the error used for the PID which is vx, vy, z where vx and
        # vy are velocities, and z is the error in the altitude of the drone
        self.pid_error = Error()

        # Initialize the 'position error to velocity error' PIDs:
        # left/right (roll) pid
        self.lr_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))
        # front/back (pitch) pid
        self.fb_pid = PIDaxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0, control_range=(-10.0, 10.0))

        # Initialize the pose callback time
        self.last_pose_time = None

        # Initialize the desired yaw velocity
        self.desired_yaw_velocity = 0

        # Initialize the current and  previous roll, pitch, yaw values
        self.current_rpy = RPY()
        self.previous_rpy = RPY()

        # initialize the current and previous states
        self.current_state = Odometry()
        self.previous_state = Odometry()

        # a variable used to determine if the drone is moving between desired positions
        self.moving = False

        # a variable that determines the maximum magnitude of the position error
        # any greater position error will overide the drone into velocity control
        self.safety_threshold = 1.5

        # determines if the position of the drone is known
        self.lost = False

        # determines if the desired poses are aboslute or relative to the drone
        self.absolute_desired_position = False

        # determines whether to use open loop velocity path planning which is
        # accomplished by calculate_travel_time
        self.path_planning = True

        # publishers
        self.cmd_pub = rospy.Publisher(
            "~commands",
            RC,
            queue_size=1
        )
        self.position_control_pub = rospy.Publisher(
            "~position_control",
            Bool,
            queue_size=1
        )
        self.heartbeat_pub = rospy.Publisher(
            "~heartbeat",
            Empty,
            queue_size=1
        )
        self._desired_height_pub = rospy.Publisher(
            "~desired/height",
            Float32,
            queue_size=1,
            latch=True
        )

        # subscribers
        rospy.Subscriber("~mode", Mode, self.current_mode_callback, queue_size=1)
        rospy.Subscriber("~state", Odometry, self.current_state_callback, queue_size=1)
        # TODO: to be refactored
        rospy.Subscriber("desired/pose", Pose, self.desired_pose_callback, queue_size=1)
        rospy.Subscriber("desired/twist", Twist, self.desired_twist_callback, queue_size=1)
        rospy.Subscriber("camera_node/lost", Bool, self.lost_callback, queue_size=1)

        # TODO: these should be services
        rospy.Subscriber("reset_transform", Empty, self.reset_callback, queue_size=1)
        rospy.Subscriber("position_control", Bool, self.position_control_callback, queue_size=1)

        # publish internal desired pose (hover pose)
        self._desired_height_pub.publish(Float32(self.desired_position.z))

    # ROS SUBSCRIBER CALLBACK METHODS
    #################################
    def current_state_callback(self, state):
        """ Store the drone's current state for calculations """
        self.previous_state = self.current_state
        self.current_state = state
        self.state_to_three_dim_vec_structs()

    def desired_pose_callback(self, msg):
        """ Update the desired pose """

        # store the previous desired position
        self.last_desired_position = self.desired_position

        # --- set internal desired pose equal to the desired pose ros message ---

        # ABSOLUTE desired x, y, z
        if self.absolute_desired_position:
            self.desired_position.x = msg.position.x
            self.desired_position.y = msg.position.y
            # the desired z must be above 0 and below the range of the ir sensor (.55meters)
            self.desired_position.z = msg.position.z if 0 <= msg.position.z <= self.max_height * 0.8 else self.last_desired_position.z

        # RELATIVE desired x, y to the CURRENT pose, but
        # RELATIVE desired z to the PREVIOUS DESIRED z (so it appears more responsive)
        else:
            self.desired_position.x = self.current_position.x + msg.position.x
            self.desired_position.y = self.current_position.y + msg.position.y
            # (doesn't limit the mag of the error)
            desired_z = self.last_desired_position.z + msg.position.z
            # the desired z must be above 0 and below the range of the ir sensor (.55meters)
            self.desired_position.z = desired_z if 0 <= desired_z <= self.max_height * 0.8 else self.last_desired_position.z

        if self.desired_position != self.last_desired_position:
            # desired pose changed, the drone should move
            self.moving = True
            print('moving')
        # publish target height
        self._desired_height_pub.publish(self.desired_position.z)

    def desired_twist_callback(self, msg):
        """ Update the desired twist """
        self.desired_velocity.x = msg.linear.x
        self.desired_velocity.y = msg.linear.y
        self.desired_velocity.z = msg.linear.z
        self.desired_yaw_velocity = msg.angular.z
        self.desired_velocity_start_time = None
        self.desired_yaw_velocity_start_time = None
        # print("Desired_velocity", self.desired_velocity)
        if self.path_planning:
            self.calculate_travel_time()

    def current_mode_callback(self, msg):
        """ Update the current mode """
        self.loginfo(f"Current mode set to: {msg.mode}")
        self.current_mode = msg.mode

    def position_control_callback(self, msg):
        """ Set whether or not position control is enabled """
        self.position_control = msg.data
        if self.position_control:
            self.desired_position = self.current_position
        if self.position_control != self.last_position_control:
            self.loginfo(f"Position control: {self.position_control}")
            self.last_position_control = self.position_control

    def reset_callback(self, _):
        """ Reset the desired and current poses of the drone and set
        desired velocities to zero """
        self.current_position = Position(z=self.current_position.z)
        self.desired_position = self.current_position
        self.desired_velocity.x = 0
        self.desired_velocity.y = 0

    def lost_callback(self, msg):
        self.lost = msg.data

    # Step Method
    def step(self):
        """ Returns the commands generated by the pid """
        self.calc_error()
        if self.position_control:
            if self.position_error.planar_magnitude() < self.safety_threshold and not self.lost:
                if self.moving:
                    if self.position_error.magnitude() > 0.05:
                        self.pid_error -= self.velocity_error * 100
                    else:
                        self.moving = False
                        print('not moving')
            else:
                self.position_control_pub.publish(False)

        if self.desired_velocity.magnitude() > 0 or abs(self.desired_yaw_velocity) > 0:
            self.adjust_desired_velocity()

        return self.pid.step(self.pid_error, self.desired_yaw_velocity)

    # HELPER METHODS
    ################
    def state_to_three_dim_vec_structs(self):
        """
        Convert the values from the state estimator into ThreeDimVec structs to
        make calculations concise
        """
        # store the positions
        pose = self.current_state.pose.pose
        self.current_position.x = pose.position.x
        self.current_position.y = pose.position.y
        self.current_position.z = pose.position.z

        # store the linear velocities
        twist = self.current_state.twist.twist
        self.current_velocity.x = twist.linear.x
        self.current_velocity.y = twist.linear.y
        self.current_velocity.z = twist.linear.z

        # store the orientations
        self.previous_rpy = self.current_rpy
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        r, p, y = tf.transformations.euler_from_quaternion(quaternion)
        self.current_rpy = RPY(r, p, y)

    def adjust_desired_velocity(self):
        """ Set the desired velocity back to 0 once the drone has traveled the
        amount of time that causes it to move the specified desired velocity
        travel distance if path_planning otherwise just set the velocities back
        to 0 after the . This is an open loop method meaning that the specified
        travel distance cannot be guarenteed. If path planning_planning is false,
        just set the velocities back to zero, this allows the user to move the
        drone for as long as they are holding down a key
        """
        curr_time = rospy.get_time()
        # set the desired planar velocities to zero if the duration is up
        if self.desired_velocity_start_time is not None:
            # the amount of time the set point velocity is not zero
            duration = curr_time - self.desired_velocity_start_time
            if duration > self.desired_velocity_travel_time:
                self.desired_velocity.x = 0
                self.desired_velocity.y = 0
                self.desired_velocity_start_time = None
        else:
            self.desired_velocity_start_time = curr_time

        # set the desired yaw velocity to zero if the duration is up
        if self.desired_yaw_velocity_start_time is not None:
            # the amount of time the set point velocity is not zero
            duration = curr_time - self.desired_yaw_velocity_start_time
            if duration > self.desired_yaw_velocity_travel_time:
                self.desired_yaw_velocity = 0
                self.desired_yaw_velocity_start_time = None
        else:
            self.desired_yaw_velocity_start_time = curr_time

    def calc_error(self):
        """
        Calculate the error in velocity, and if in position hold, add the
        error from lr_pid and fb_pid to the velocity error to control the
        position of the drone
        """
        # store the time difference
        pose_dt = 0
        if self.last_pose_time is not None:
            pose_dt = rospy.get_time() - self.last_pose_time
        self.last_pose_time = rospy.get_time()
        # calculate the velocity error
        self.velocity_error = self.desired_velocity - self.current_velocity
        # calculate the z position error
        dz = self.desired_position.z - self.current_position.z
        # calculate the pid_error from the above values
        self.pid_error.x = self.velocity_error.x
        self.pid_error.y = self.velocity_error.y
        self.pid_error.z = dz
        # multiply by 100 to account for the fact that code was originally written using cm
        self.pid_error = self.pid_error * 100
        if self.position_control:
            # calculate the position error
            self.position_error = self.desired_position - self.current_position
            # calculate a value to add to the velocity error based based on the
            # position error in the x (roll) direction
            lr_step = self.lr_pid.step(self.position_error.x, pose_dt)
            # calculate a value to add to the velocity error based based on the
            # position error in the y (pitch) direction
            fb_step = self.fb_pid.step(self.position_error.y, pose_dt)
            self.pid_error.x += lr_step
            self.pid_error.y += fb_step

    def calculate_travel_time(self):
        """ return the amount of time that desired velocity should be used to
        calculate the error in order to move the drone the specified travel
        distance for a desired velocity
        """
        if self.desired_velocity.magnitude() > 0:
            # tiime = distance / velocity
            travel_time = self.desired_velocity_travel_distance / self.desired_velocity.planar_magnitude()
        else:
            travel_time = 0.0
        self.desired_velocity_travel_time = travel_time

    def reset(self):
        """ Set desired_position to be current position, set
        filtered_desired_velocity to be zero, and reset both the PositionPID
        and VelocityPID
        """
        # reset position control variables
        self.position_error = Error(0, 0, 0)
        self.desired_position = Position(self.current_position.x, self.current_position.y, self.hover_height)
        # reset velocity control_variables
        self.velocity_error = Error(0, 0, 0)
        self.desired_velocity = Velocity(0, 0, 0)
        # reset the pids
        self.pid.reset()
        self.lr_pid.reset()
        self.fb_pid.reset()

    def publish_cmd(self, cmd):
        """ Publish the controls """
        msg = RC()
        msg.roll = cmd[0]
        msg.pitch = cmd[1]
        msg.yaw = cmd[2]
        msg.throttle = cmd[3]
        self.cmd_pub.publish(msg)


def main(controller_class):
    # Verbosity between 0 and 2, 2 is most verbose
    verbose = 2

    # create the PIDController object
    pid = controller_class()

    # set the loop rate (Hz)
    loop_rate = rospy.Rate(pid.frequency)
    print('PID Controller Started')

    while not pid.is_shutdown:
        pid.heartbeat_pub.publish(Empty())

        # Steps the PID. If we are not flying, this can be used to
        # examine the behavior of the PID based on published values
        fly_command = pid.step()

        # reset the pids after arming and start up in velocity control
        if pid.previous_mode == 0:  # 'DISARMED'
            if pid.current_mode == 1:  # 'ARMED'
                pid.reset()
                pid.position_control_pub.publish(False)
                # ---
                pid.loginfo("Detected state change: DISARMED -> ARMED")
                pid.previous_mode = pid.current_mode

        # reset the pids right before flying
        if pid.previous_mode == 1:  # 'ARMED'
            if pid.current_mode == 2:  # 'FLYING'
                pid.reset()
                # ---
                pid.loginfo("Detected state change: ARMED -> FLYING")
                pid.previous_mode = pid.current_mode

            elif pid.current_mode == 0:
                # ---
                pid.loginfo("Detected state change: ARMED -> DISARMED")
                pid.previous_mode = pid.current_mode

        # if the drone is flying, send the fly_command
        elif pid.previous_mode == 2:  # 'FLYING'
            if pid.current_mode == 2:  # 'FLYING'
                # Safety check to ensure drone does not fly too high height_safety_here
                if pid.current_state.pose.pose.position.z > pid.max_height:
                    print("\n disarming because drone is too high \n")
                    break
                # Publish the ouput of pid step method
                pid.publish_cmd(fly_command)

            # after flying, take the converged low i terms and set these as the
            # initial values, this allows the drone to "learn" and get steadier
            # with each flight until it converges
            # NOTE: do not store the throttle_low.init_i or else the drone will
            # take off abruptly after the first flight
            elif pid.current_mode == 0:  # 'DISARMED'
                pid.pid.roll_low.init_i = pid.pid.roll_low.integral
                pid.pid.pitch_low.init_i = pid.pid.pitch_low.integral
                # Uncomment below statements to print the converged values.
                # Make sure verbose = 0 so that you can see these values
                if verbose >= 2:
                    print('roll_low.init_i', pid.pid.roll_low.init_i)
                    print('pitch_low.init_i', pid.pid.pitch_low.init_i)
                # ---
                pid.loginfo("Detected state change: FLYING -> DISARMED")
                pid.previous_mode = pid.current_mode

        if verbose >= 2:
            if pid.position_control:
                print('current position:', pid.current_position)
                print('desired position:', pid.desired_position)
                print('position error:', pid.position_error)
            else:
                print('current velocity:', pid.current_velocity)
                print('desired velocity:', pid.desired_velocity)
                print('velocity error:  ', pid.velocity_error)
            print('pid_error:       ', pid.pid_error)
            print('r,p,y,t:', fly_command)
            print('throttle_low._i', pid.pid.throttle_low.integral)
            print('throttle._i', pid.pid.throttle.integral)

        if verbose >= 1:
            error = pid.pid_error
            print(
                "Errors:",
                "\t Z: ", str(error.z)[:5],
                "\t X ", str(error.x)[:5],
                "\t Y ", str(error.y)[:5]
            )
            print("---------------------------------------")
        # ---
        loop_rate.sleep()


if __name__ == '__main__':
    main(PIDController)
