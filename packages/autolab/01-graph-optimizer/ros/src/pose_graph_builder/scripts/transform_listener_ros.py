#!/usr/bin/env python3
import os
import random
import threading
import signal
import sys
import yaml
from multiprocessing import Process
import duckietown_cslam.resampler.resampler as resampler
from duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder import create_info_matrix
import subprocess
import time
import g2o
import geometry as g
import numpy as np
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import *
from std_msgs.msg import Header, String, Time
from visualization_msgs.msg import *
from nav_msgs.msg import Odometry, Path
from duckietown_msgs.msg import AprilTagExtended
import inspect
import yaml


def load_yaml_file(fn):
    with open(fn) as f:
        data = f.read()
    return yaml.load(data, Loader=yaml.SafeLoader)


def get_node_type(node_id):
    if "watchtower" in node_id:
        node_type = "watchtower"
    elif "apriltag" in node_id:
        node_type = "apriltag"
    else:
        node_type = "duckiebot"
    return node_type


def get_transform_from_data(data):
    t = [
        data.transform.translation.x, data.transform.translation.y,
        data.transform.translation.z
    ]

    # Create rotation matrix. NOTE: in Pygeometry, quaternion is
    # the (w, x, y, z) form.
    q = [
        data.transform.rotation.w, data.transform.rotation.x,
        data.transform.rotation.y, data.transform.rotation.z
    ]
    M = g.rotations.rotation_from_quaternion(np.array(q))

    return g2o.Isometry3d(M, t)


def create_info_matrix_odometry(standard_space_deviation, standard_angle_deviation, constraints=[1, 1, 1, 1, 1, 1]):
    m = np.eye(6)
    for i in range(0, 3, 2):
        if(not constraints[i]):
            m[i, i] = 0
        else:
            m[i, i] = 1.0 / (standard_space_deviation**2)
    m[1, 1] *= 5
    for i in range(3, 6):
        if(not constraints[i]):
            m[i, i] = 0
        else:
            m[i, i] = 1.0 / (standard_angle_deviation**2)
    return m

# def create_info_matrix(standard_measure_deviation, standard_angle_deviation, constraints=[1, 1, 1, 1, 1, 1]):
#     m = np.eye(6)
#     for i in range(0, 3):
#         if(not constraints[i]):
#             m[i, i] = 0
#         else:
#             m[i, i] = 1 / (standard_measure_deviation**2)
#     for i in range(3, 6):
#         if(not constraints[i]):
#             m[i, i] = 0
#         else:
#             m[i, i] = 1.0 / \
#                 (np.sin(np.deg2rad(standard_angle_deviation))**2)

#     return m


class PointBroadcaster(threading.Thread):
    def __init__(self, dictionnary):
        threading.Thread.__init__(self)
        self.pose_dict = dictionnary
        self.br = tf2_ros.TransformBroadcaster()
        self.second_broadcaster = rospy.Publisher(
            "/agent_poses", geometry_msgs.msg.TransformStamped, queue_size=1)

    def tfbroadcast(self, node_id, node_pose):
        """ Brodcasts a node in the tree of transforms with TF.

            Args:
                node_id: ID of the node.
                node_pose: Pose of the node.
        """
        # Create broadcaster and transform.
        if(node_pose == None):
            print(("No pose given for %s" % node_id))
            return
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        # Set frame ID. TODO: change it depending on the node type.
        if (node_id.startswith("duckiebot")):
            t.header.frame_id = "map"
            t.child_frame_id = node_id

        else:
            t.header.frame_id = "map"
            t.child_frame_id = node_id

        # Set child frame ID.
        # Set transform:
        # - Create translation vector.
        t.transform.translation.x = node_pose.t[0]
        t.transform.translation.y = node_pose.t[1]
        t.transform.translation.z = node_pose.t[2]
        # - Create rotation matrix.
        #   Verify that the rotation is a proper rotation.
        # det = np.linalg.det(node_pose.R)
        # if (det < 0):
        #     print("after optim : det = %f" % det)
        #   NOTE: in Pygeometry, quaternion is the (w, x, y, z) form.
        R = node_pose.R

        # R[np.where(np.abs(R) < 0.00001)] = 0
        # q = [1, 0, 0, 0]
        try:
            q = g.rotations.quaternion_from_rotation(R)
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
            # Send the transform.
            self.br.sendTransform(t)
            if not t.child_frame_id.startswith("apriltag"):
                self.second_broadcaster.publish(t)
        except:
            print(("bad rotation %s" % str(R)))
        # print("Proportion sendTransform/total fonction : %f" % ((c-b)/(c-a)))
        # print("Proportion quaternion/total fonction : %f" % ((f-e)/(c-a)))

    def run(self):
        for node_id, node_pose in self.pose_dict.items():
            if node_pose != -1:
                self.tfbroadcast(node_id, node_pose)


class PathBroadcaster(threading.Thread):
    def __init__(self, node_path, node_id):
        threading.Thread.__init__(self)
        self.node_path = node_path
        self.node_id = node_id
        self.path = Path()
        self.publisher = rospy.Publisher(
            '/movable_path_%s' % self.node_id, Path, queue_size=10)

    def run(self):
        """ Brodcasts the path for the node

            Args:
                node_id: ID of the node.
                node_path: Path of the node. Dictionnary of {timestamp : g2oTransform}
        """
        # Create broadcaster and transform.
        # Set frame ID. TODO: change it depending on the node type.
        self.path.header.stamp = rospy.Time.now()
        if (self.node_id.startswith("duckiebot")):
            self.path.header.frame_id = "map"
        else:
            self.path.header.frame_id = "map"

        for time_stamp in sorted(self.node_path.keys()):
            node_pose = self.node_path[time_stamp]
            pose_stamped = geometry_msgs.msg.PoseStamped()
            R = node_pose.R
            # R[np.where(np.sabs(R) < 0.00001)] = 0
            # q = [1, 0, 0, 0]

            try:
                q = g.rotations.quaternion_from_rotation(R)

                pose = Pose()
                pose.position.x = node_pose.t[0]
                pose.position.y = node_pose.t[1]
                pose.position.z = node_pose.t[2]
                # NOTE: in Pygeometry, quaternion is the(w, x, y, z) form.
                pose.orientation.w = q[0]
                pose.orientation.x = q[1]
                pose.orientation.y = q[2]
                pose.orientation.z = q[3]
                # print("-------------------------- Pose", pose)
                pose_stamped.pose = pose
                pose_stamped.header.stamp.secs = int(time_stamp)
                pose_stamped.header.stamp.nsecs = int((
                    time_stamp - int(time_stamp)) * 10**9)

                # print("-------------------------- PoseStamped", pose_stamped)
                # point.z = 0.001
                self.path.poses.append(pose_stamped)
                # print("-------------------------- SELF.PATH", self.path)
            except:
                print(("bad rotation %s" % str(R)))

        self.publisher.publish(self.path)


class TransformListener():
    """Listens for the transforms published by the acquisition node, associates
       each object in the map to an ID, builds an internal pose graph and
       broadcasts the (optimized) tree of transforms, to be e.g. visualized with
       duckietown-visualization.

       Attributes:
           resampler: A resampler to send better data to the graph.
           old_odometry_stamps: Stores, for each ID, the time stamp of the last
                                odometry message read for the object with that
                                ID.
           id_map: Contains all April tags, mapped to their ID and type based on
                   the database read.
           last_callback: Stores the time when the last callback was started.
           optim_period: Sets the time (in seconds) that should pass before a
                         new optimization step is performed.
           optim_period_counter: Stores the time that one should look at to
                                 decide whether to perform a new optimization
                                 step.
           num_messages_received: Stores the number of transform messages
                                  received. This is used to ensure that
                                  optimization is performed only if enough
                                  messages have been received (i.e. if the graph
                                  contains enough information).
           edge_counters: Store the number of edges between watchotowers and april tags.
                          This is used to prevent adding too many times the same edge to the graph
    """

    def __init__(self):
        self.max_iteration = rospy.get_param("maximum_g2o_iterations")
        self.minimum_edge_number_for_optimization = rospy.get_param(
            "minimum_edge_number_for_optimization")
        self.rejection_sampling_int = rospy.get_param("rejection_sampling_int")
        self.max_number_same_edge = rospy.get_param("max_number_same_edge")
        self.verbose = rospy.get_param("optim_verbose")
        self.save_output = rospy.get_param("save_g2o_output")
        self.optimization_frequency = rospy.get_param("optimization_frequency")
        self.online_optimization = rospy.get_param("online_optimization")

        self.num_messages_received = 0
        self.edge_counters = dict()
        self.optimization_period = 1.0/self.optimization_frequency
        self.resampler = None
        self.forward_id_map = {}
        self.pose_errors = []
        self.first_time_stamp = -1
        self.lastbeat = -1
        self.timeout = 10
        self.callback_times = []
        self.threads = []
        self.bag_reader = None
        self.got_bag = False

    def initialize_id_map(self):
        """ Loads April tags into the ID map, assigning each tag in the database
            its ID and its type (e.g. TrafficSign, Localization, etc.).
        """
        config_folder = rospy.get_param("config_folder")
        duckietown_world_folder = rospy.get_param("duckietown_world_folder")
        aprilstagDB = "%s/%s" % (duckietown_world_folder,
                                 "src/duckietown_world/data/apriltagsDB.yaml")
        aprilstagDB_custom = "%s/%s" % (config_folder,
                                        "apriltagsDB_custom.yaml")
        # Read YAML file.
        for apriltagfile in [aprilstagDB, aprilstagDB_custom]:
            if os.path.isfile(apriltagfile):

                with open(apriltagfile, 'r') as stream:
                    try:
                        complete_dict = yaml.safe_load(stream)
                        for myobject in complete_dict:
                            tag_id = myobject["tag_id"]
                            mytype = myobject['tag_type']
                            vehicle_name = myobject['vehicle_name']
                            self.forward_id_map[str(tag_id)] = (
                                mytype, vehicle_name)
                            if not vehicle_name and mytype == "vehicle":
                                # print(vehicle_name)
                                print("A vehicle doesn't have a name")

                    except yaml.YAMLError as exc:
                        print(exc)
            else:
                print(("apriltagDB file not found at %s" % apriltagfile))

    def find_vertex_name(self, id):
        """ Returns the format ID of an object in the ID map based on its type.

            Args:
                id: Original ID of the object in the ID map.

            Returns:
                ID of the objected, formatted by adding "duckiebot_" or "apriltag_"
                to it, based on its type.
        """
        if id.isdigit():
            tag_type, vehicle_name = self.forward_id_map.get(
                id, ("apriltag", None))
            if tag_type == "Vehicle":
                id = vehicle_name
            else:
                id = "apriltag%s" % id

        return id

    def handle_odometry_message(self, node_id, data, time_stamp):
        """Processes an odometry message, adding an edge to the graph and
           keeping track of the last time stamp before each new odometry message
           (needed to handle edges in the pose graph and connect nodes).

           Args:
               node_id: ID of the object sending the odometry message.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """

        transform = get_transform_from_data(data)
        space_dev = self.default_variance["odometry"]["position_deviation"]
        angle_dev = self.default_variance["odometry"]["angle_deviation"]
        measure_information = create_info_matrix_odometry(space_dev, angle_dev)
        # Add edge to the graph.
        return self.resampler.handle_odometry_edge(node_id, transform, time_stamp, measure_information)

    def handle_watchtower_message(self, node_id0, node_id1, data, time_stamp, pose_error=None):
        """Processes a message containing the pose of an object seen by a
           watchtower and adds an edge to the graph. If the object seen is a
           Duckiebot, adjusts the pose accordingly.

           Args:
               node_id0: ID of the object (watchtower) that sees the April tag of the
                    other object.
               node_id1: ID of the object whose April tag is seen by the watchtower.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        transform = get_transform_from_data(data)
        # Get type of the object seen.

        type_of_object_seen = get_node_type(node_id1)
        space_dev = self.default_variance["watchtower"]["position_deviation"]
        angle_dev = self.default_variance["watchtower"]["angle_deviation"]
        if(pose_error != None):
            measure_information = create_info_matrix(
                space_dev * pose_error, angle_dev * pose_error)
        else:
            measure_information = create_info_matrix(space_dev, angle_dev)

        if (type_of_object_seen == "duckiebot"):
            # print("watzchtower %s is seing duckiebot %s" %
                #   (node_id0, node_id1))
            # In case of Duckiebot the pose needs to be adjusted to take into
            # account the pose of the April tag w.r.t. the base frame of the
            # Duckiebot.
            t = [0.0, 0.0, 0.055]
            z_angle = 90
            x_angle = 178
            z_angle = np.deg2rad(z_angle)
            x_angle = np.deg2rad(x_angle)
            R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
            R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
            R = np.matmul(R_x, R_z)  # verified!
            H_apriltag_to_base = g2o.Isometry3d(R, t)
            transform = transform * H_apriltag_to_base.inverse()

            # Add edge to the graph.
            return self.resampler.handle_watchtower_edge(node_id0, node_id1, transform, time_stamp, measure_information=measure_information, is_duckiebot=True)
        else:
            # Add edge to the graph.
            if node_id0 not in self.edge_counters:
                self.edge_counters[node_id0] = dict()
            if node_id1 not in self.edge_counters[node_id0]:
                self.edge_counters[node_id0][node_id1] = 0

            if(self.edge_counters[node_id0][node_id1] < self.max_number_same_edge):
                self.resampler.handle_watchtower_edge(
                    node_id0, node_id1, transform, time_stamp)
                self.edge_counters[node_id0][node_id1] += 1
            else:
                a = random.randint(0, self.rejection_sampling_int)
                if(a == 0):
                    self.edge_counters[node_id0][node_id1] += 1

                    return self.resampler.handle_watchtower_edge(node_id0, node_id1, transform, time_stamp, measure_information=measure_information)

    def handle_duckiebot_message(self, node_id0, node_id1, data, time_stamp, pose_error=None):
        """Processes a message containing the pose of an object seen by a
           Duckiebot and adds an edge to the graph. Note: we assume that a
           Duckiebot cannot see the April tag of another Duckiebot, so no
           adjustment based on the object seen is needed.

           Args:
               node_id0: ID of the object (Duckiebot) that sees the April tag of the
                    other object.
               node_id1: ID of the object whose April tag is seen by the Duckiebot.
               transform: Transform contained in the ROS message.
               time_stamp: Timestamp associated to the ROS message.
        """
        transform = get_transform_from_data(data)
        space_dev = self.default_variance["duckiebot"]["position_deviation"]
        angle_dev = self.default_variance["duckiebot"]["angle_deviation"]
        if(pose_error != None):
            measure_information = create_info_matrix(
                space_dev * pose_error, angle_dev * pose_error)
        else:
            measure_information = create_info_matrix(space_dev, angle_dev)

        # Get type of the object that sees the other object, for a sanity check.

        # The pose needs to be adjusted to take into account the relative
        # pose of the camera on the Duckiebot w.r.t. to the base frame of
        # the Duckiebot.
        t = [0.075, 0.0, 0.025]
        # This angle is an estimate of the angle by which the plastic
        # support that holds the camera is tilted.
        x_angle = -102
        z_angle = -90
        x_angle = np.deg2rad(x_angle)
        z_angle = np.deg2rad(z_angle)
        R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)

        R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
        R = np.matmul(R_z, R_x)  # verified
        H_base_to_camera = g2o.Isometry3d(R, t)
        transform = H_base_to_camera * transform
        # Add edge to the graph.
        # return self.resampler.handle_duckiebot_edge(node_id0, node_id1, transform, time_stamp, measure_information)

    def filter_name(self, node_id):
        """ Converts the frame IDs of the objects in the ROS messages (e.g.,
            Duckiebots, watchtowers, etc.) to the format <type>_<tag_id>, where
            <type> should be one of the types defined in DuckietownGraphBuilder
            (e.g. "duckiebot", "watchtower", "apriltag") and <tag_id> is the ID of
            the April tag of the object in the ID map.

            Args:
                node_id: Frame ID in the ROS message, to be converted.

            Returns:
                Converted frame ID.
        """
        if ("watchtower" not in node_id):
            node_id = self.find_vertex_name(node_id)

        return node_id

    def transform_callback(self, data, msg_type):
        """ ROS callback.
        """
        a = time.time()

        self.num_messages_received += 1
        self.lastbeat = a
        # Get frame IDs of the objects to which the ROS messages are referred.
        node_id0 = data.header.frame_id
        if msg_type == "AprilTagExtended":
            node_id1 = str(data.tag_id)
            # pose_error = float(data.pose_error) * 10**8 / 37.0
            # self.pose_errors.append(pose_error)
            # var = np.var(self.pose_errors)
            # mean = np.mean(self.pose_errors)
            # print("pose error : mean %f, var %f" % (mean, var))
        elif msg_type == "TransformStamped":
            node_id1 = data.child_frame_id
        else:
            raise Exception(
                "Transform callback received unsupported msg_type %s" % msg_type)

        # Convert the frame IDs to the right format.
        # print("before filter : message from %s to %s" % (node_id0, node_id1))

        node_id0 = self.filter_name(node_id0)
        node_id1 = self.filter_name(node_id1)
        # print("after filter : message from %s to %s" % (node_id0, node_id1))
        # Ignore messages from one watchtower to another watchtower (i.e.,
        # odometry messages between watchtowers).
        is_from_watchtower = False
        if ("watchtower" in node_id0):
            is_from_watchtower = True
            if ("watchtower" in node_id1):
                print(data)
                return 0

        # Create translation vector.
        header_time = Time(data.header.stamp)
        time_stamp = header_time.data.secs + header_time.data.nsecs * 10**(-9)

        if self.first_time_stamp == -1:
            self.first_time_stamp = time_stamp
            self.resampler.signal_reference_time_stamp(self.first_time_stamp)

        if (node_id1 == node_id0):
            # print("got odometry")
            # Same ID: odometry message, e.g. the same Duckiebot sending
            # odometry information at different instances in time.
            self.handle_odometry_message(
                node_id1, data, time_stamp)
        elif (is_from_watchtower):
            # print("got message from watchtower")

            # Tag detected by a watchtower.
            self.handle_watchtower_message(
                node_id0, node_id1, data, time_stamp)
        else:
            # Tag detected by a Duckiebot.
            # print("got message from duckiebot")
            self.handle_duckiebot_message(
                node_id0, node_id1, data, time_stamp)
        b = time.time()
        self.callback_times.append(b-a)

    def optimization_callback(self, timer_event):
        if (self.num_messages_received >= self.minimum_edge_number_for_optimization):
            # a = rospy.get_time()
            self.resampler.optimize(
                self.max_iteration,
                save_result=self.save_output,
                verbose=self.verbose,
                output_name="/tmp/output.g2o",
                online=self.online_optimization)
            # b = rospy.get_time()

            # Broadcast tree of transforms with TF.
            pose_dict = self.resampler.get_all_optimized_poses()
            # c = rospy.get_time()

            point_broadcaster = PointBroadcaster(pose_dict)
            point_broadcaster.start()

            path_dict = self.resampler.get_optimized_movable_paths()
            # print(path_dict)
            for node_id, node_path in path_dict.items():
                path_broadcaster = PathBroadcaster(node_path, node_id)
                path_broadcaster.start()

    def heartbeat_callback(self, timer_event):
        if not self.got_bag:
            return
        # if(self.callback_times != []):
        #     mean = np.mean(self.callback_times)
        #     var = np.var(self.callback_times)
        #     print("callback : mean = %f and var = %f" % (mean, var))
        current_time = time.time()
        if self.lastbeat == -1:
            return
        diff = current_time - self.lastbeat
        print("heartbeat accu:", diff)
        self.timeout = 30
        if diff > self.timeout:
            print("heartbeat is timed out")
            self.resampler.wait()
            self.resampler.optimize(
                self.max_iteration * 10,
                save_result=self.save_output,
                verbose=self.verbose,
                output_name="/tmp/output.g2o",
                online=self.online_optimization,
                final=True)
            pose_dict = self.resampler.get_all_optimized_poses()
            # c = rospy.get_time()

            point_broadcaster = PointBroadcaster(pose_dict)
            point_broadcaster.start()

            path_dict = self.resampler.get_optimized_movable_paths()
            # print(path_dict)
            path_broadcasters = []
            for node_id, node_path in path_dict.items():
                path_broadcaster = PathBroadcaster(node_path, node_id)
                path_broadcaster.start()
                path_broadcasters.append(path_broadcaster)

            point_broadcaster.join()
            for path_broadcaster in path_broadcasters:
                path_broadcaster.join()
            self.signal_handler(signal.SIGINT, inspect.currentframe())

    def listen(self):
        """Initializes the graph based on the floor map and initializes the ID
           map. Then starts listening to the poses and odometry topics published
           by the acquistion node.
        """
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        map_name = os.getenv("MAP_NAME", "robotarium2")

        if ".yaml" not in map_name and ".yml" not in map_name:
            map_name = "%s.yaml" % map_name

        initial_floor_april_tags = "%s/%s/%s" % (rospy.get_param("duckietown_world_folder"),
                                                 "src/duckietown_world/data/gd1/maps", map_name)
        priors_filename = "%s/%s" % (rospy.get_param("config_folder"),
                                     "priors.yaml")
        default_variance_filename = "%s/%s" % (rospy.get_param("config_folder"),
                                               "default_variance.yaml")

        self.default_variance = load_yaml_file(default_variance_filename)
        stocking_time = rospy.get_param("stocking_time")
        using_priors = rospy.get_param("using_priors")
        result_folder = rospy.get_param("result_folder")
        resampling_frequency = rospy.get_param("resampling_frequency", 20.0)
        bag_is_present = False
        bag_env = "ATMSGS_BAG"
        if not bag_env in os.environ:
            print(('I expect environment variable %s' % bag_env))
        else:
            bag_path = os.environ[bag_env]
            bag_is_present = True

        # Build graph based on floor map.
        self.resampler = resampler.Resampler(initial_floor_april_tags=initial_floor_april_tags, stocking_time=stocking_time, priors_filename=priors_filename,
                                             default_variance=self.default_variance, using_priors=using_priors, result_folder=result_folder, resampling_frequency=resampling_frequency)
        # Initialize ID map.
        self.initialize_id_map()
        # Subscribe to topics.
        rospy.Subscriber("/poses_acquisition/poses", AprilTagExtended,
                         lambda msg: self.transform_callback(msg, "AprilTagExtended"))
        rospy.Subscriber("/poses_acquisition/odometry", TransformStamped,
                         lambda msg: self.transform_callback(msg, "TransformStamped"))

        # Create a regular callback to invoke optimization on a regular basis
        self.optim_callback = rospy.Timer(rospy.Duration(self.optimization_period),
                                          self.optimization_callback)

        self.heartbeat = rospy.Timer(rospy.Duration(1.0),
                                     self.heartbeat_callback)
        # spin() simply keeps python from exiting until this node is stopped
        if bag_is_present:
            self.read_bag(bag_path)
        rospy.spin()

    def read_bag(self, bag_path):
        if os.path.isfile(bag_path):
            self.got_bag = True
            rospy.sleep(4)
            try:
                print(bag_path)
                subprocess.check_output("rosbag play %s" %
                                        bag_path, shell=True)
                print("finished with reading the bag")
            except Exception as e:
                print(("error : %s" % e))

        else:
            self.got_bag = False

            print("No bag to process. Will wait for it.")

    def signal_handler(self, sig, frame):
        if self.bag_reader != None:
            self.bag_reader.join()
        print('You pressed Ctrl+C! Experiment will be saved and ended')
        self.resampler.on_shutdown()
        print("Exiting transform listener")
        self.optim_callback.shutdown()
        self.heartbeat.shutdown()
        rospy.signal_shutdown(
            "Shutting down after cleaning and recording data")
        print("Every thing should be down")


def main():
    rospy.init_node('listener', anonymous=True, disable_signals=True)

    tflistener = TransformListener()
    # rospy.on_shutdown(tflistener.on_shutdown)
    signal.signal(signal.SIGINT, tflistener.signal_handler)
    signal.signal(signal.SIGTERM, tflistener.signal_handler)

    tflistener.listen()
    # rospy.signal_shutdown(reason)


if __name__ == '__main__':
    main()
