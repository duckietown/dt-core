import random
import threading
import csv
import yaml
import os
import time

import duckietown_cslam.g2oGraphBuilder.g2ograph_builder as g2oGB

import g2o
import geometry as g
import numpy as np
import pprint


def get_node_type(node_id):
    if "watchtower" in node_id:
        node_type = "watchtower"
    elif "apriltag" in node_id:
        node_type = "apriltag"
    else:
        node_type = "duckiebot"
    return node_type


def interpolate_measure(measure, alpha):
    R = measure.R
    t = measure.t
    q = g.SE3_from_rotation_translation(R, t)
    vel = g.SE3.algebra_from_group(q)

    rel = g.SE3.group_from_algebra(vel * alpha)
    newR, newt = g.rotation_translation_from_SE3(rel)
    return g2o.Isometry3d(newR, newt)


def create_info_matrix(standard_space_deviation, standard_angle_deviation, constraints=[1, 1, 1, 1, 1, 1]):
    m = np.eye(6)
    for i in range(0, 3):
        if(not constraints[i]):
            m[i, i] = 0
        else:
            m[i, i] = 1 / (standard_space_deviation**2)
    for i in range(3, 6):
        if(not constraints[i]):
            m[i, i] = 0
        else:
            m[i, i] = 1.0 / (standard_angle_deviation**2)
    return m


class Prior(object):
    def __init__(self, name, from_type, to_type, position, rotation, constraint_list, standard_space_deviation, standard_angle_deviation):
        self.name = name
        self.from_type = from_type
        self.to_type = to_type
        self.position = position
        self.constraints = constraint_list
        self.rotation = rotation
        self.measure = None
        self.measure_information = None
        self.from_origin = False
        self.standard_space_deviation = standard_space_deviation
        self.standard_angle_deviation = standard_angle_deviation
        self.create_measure()
        self.create_info_matrix()
        if self.from_type == "origin":
            self.from_origin = True

    def create_measure(self):
        t = self.position

        roll_angle = np.deg2rad(self.rotation[0])
        pitch_angle = np.deg2rad(self.rotation[1])
        yaw_angle = np.deg2rad(self.rotation[2])

        R_roll = g.rotation_from_axis_angle(np.array([0, 1, 0]), roll_angle)
        R_pitch = g.rotation_from_axis_angle(np.array([1, 0, 0]), pitch_angle)
        R_yaw = g.rotation_from_axis_angle(np.array([0, 0, 1]), yaw_angle)

        R = np.matmul(R_roll, np.matmul(R_pitch, R_yaw))
        self.measure = g2o.Isometry3d(R, t)

    def create_info_matrix(self):
        self.measure_information = create_info_matrix(
            self.standard_space_deviation, self.standard_angle_deviation, self.constraints)

    def is_from_origin(self):
        return self.from_origin

    def unary_concerns(self, node_type):
        if not self.from_origin:
            print("WARNING : binary prior asked for unary check")
            return False
        if self.to_type == node_type:
            return True
        return False

    def binary_concerns(self, node0_type, node1_type):
        if self.from_origin:
            print("WARNING : unary prior asked for binary check")
            return False
        if self.from_type == node0_type and self.to_type == node1_type:
            return True
        return False

    def __str__(self):
        string = 'prior : %s\n' % self.name
        string += '\tfrom %s to %s\n' % (self.from_type, self.to_type)
        string += '\tposition: %s\n' % str(self.position)
        string += '\trotation: %s\n' % str(self.rotation)
        string += '\tconstraint: %s\n' % str(self.constraints)
        return string


class PriorHandler(object):
    def __init__(self, priors_filename=None):
        self.priors_filename = priors_filename
        self.prior_list = []
        # load priors from config file
        self.load_priors()

    def load_priors(self):
        with open(self.priors_filename, 'r') as priors_file:
            priors = yaml.load(priors_file)
            for prior_name, prior in list(priors.items()):
                from_type = prior['from_type']
                to_type = prior['to_type']
                position = prior['position']
                rotation = prior['rotation']
                constraints = prior['constraints']
                standard_space_deviation = prior['standard_space_deviation']
                standard_angle_deviation = prior['standard_angle_deviation']
                constraint_list = [constraints['x'], constraints['y'], constraints['z'],
                                   constraints['qx'], constraints['qy'], constraints['qz']]
                new_prior = Prior(prior_name, from_type, to_type,
                                  position, rotation, constraint_list, standard_space_deviation, standard_angle_deviation)
                self.prior_list.append(new_prior)
                print((str(new_prior)))


class CyclicCounter(object):
    def __init__(self, total_size, chunck_size, node_id):
        self.total_size = total_size
        self.chunck_size = chunck_size
        self.front_pointer = 0
        self.back_pointer = -1
        self.mode = 0
        self.mode_types = ["back_to_front", "front_to_back"]
        self.removed_indices = []
        self.node_id = node_id

    def next_index(self):
        index = self.front_pointer

        # handle index increment according to mode
        if (self.mode == 0):
            # This means that front_pointer is ahead of back pointer (we have not cycled yet)

            if(self.front_pointer + 1 < self.total_size):
                self.front_pointer += 1
            else:
                if(self.back_pointer > 0):
                    self.front_pointer = 0
                    self.mode = 1
                else:
                    self._clean_counter()
                    # TODO : handle if mode changes!!

                    if(self.back_pointer > 0):
                        self.front_pointer = 0
                        self.mode = 1
                    else:
                        print(
                            "[ERROR] Cyclic counter cannot cycle as back pointer is still at 0. Need cleaning")

        else:
            if (self.front_pointer + 1 < self.back_pointer):
                self.front_pointer += 1
            else:
                self._clean_counter()
                # TODO : handle if mode changes!!
                if (self.front_pointer + 1 < self.back_pointer):
                    self.front_pointer += 1
                else:
                    print(
                        "[ERROR] Cyclic counter front pointer has reached back pointer. Need cleaning")

        return index

    def remove_index(self, index_to_remove):
        # security checks
        if(index_to_remove in self.removed_indices):
            print(
                "[ERROR] cyclic counter: index_to_remove was already added but not yet freed!")
            return -1

        self.removed_indices.append(index_to_remove)

        if len(self.removed_indices) >= self.chunck_size:
            self._clean_counter()

    def _clean_counter(self):
        # print("Cleaning cyclic counter for %s : \n front_pointer is %d\n back_pointer is %d\n removed_indices is" % (self.node_id, self.front_pointer, self.back_pointer) + str(self.removed_indices))
        marked_freed = []
        self.removed_indices.sort()
        for i in self.removed_indices:
            if i == self.back_pointer + 1:
                self.back_pointer += 1
                if self.back_pointer == self.total_size - 1:
                    self.back_pointer = -1
                    self.modes = 0
                marked_freed.append(i)
            elif i > self.back_pointer + 1:
                break
        for i in marked_freed:
            self.removed_indices.remove(i)
        # print("After cleaning cyclic counter for %s : \n front_pointer is %d\n back_pointer is %d\n removed_indices is" % (self.node_id, self.front_pointer, self.back_pointer) + str(self.removed_indices))


class Node(object):
    def __init__(self, node_id, types, duckietown_graph, movable=False):
        self.node_id = node_id
        self.node_type = get_node_type(node_id)
        print("::::::::::::::: NODE_ID", node_id)
        self.node_number = int(list(filter(str.isdigit, node_id))[0])
        self.types = types
        self.duckietown_graph = duckietown_graph
        self.time_stamps_to_indices = dict()
        self.last_time_stamp = None
        self.node_lock = g2oGB.ControlableLock()
        self.movable = movable
        if(self.node_type not in self.types):
            print(("Registering node of type %s unkown in types = " %
                  self.node_type + str(self.types)))

    def get_g2o_index(self, time_stamp):
        """Given a timestamp associated to that node, outputs an integer that can be used as a
           index for the node in g2o (the latter only handles integer indices
           for the nodes).

           Args:
              time_stamp: Timestamp.

           Returns:
              Input ID converted to an integer that can be used as an index by
              g2o.
        """
        b = int(self.node_number) % 1000
        a = self.types.index(self.node_type) + 1
        result = a * 10**8 + b * 10**5
        return result

    def is_movable(self):
        return False

    def add_timestamp(self, time_stamp):
        with self.node_lock:
            if self.time_stamps_to_indices == {}:
                self.time_stamps_to_indices = {time_stamp: 0}
                self.last_time_stamp = time_stamp
                return True
            else:
                return False

    def get_last_known_position(self):
        vertex_id = self.get_g2o_index(self.last_time_stamp)
        if vertex_id == -1:
            return -1
        vertex_pose = self.duckietown_graph.get_vertex_pose(vertex_id)
        return vertex_pose


class MovableNode(Node):
    def __init__(self, node_id, types, duckietown_graph, retro_interpolate=False, stocking_time=None, result_folder="/tmp"):
        super(MovableNode, self).__init__(
            node_id, types, duckietown_graph, movable=True)
        self.retro_interpolate = retro_interpolate
        self.first_odometry_time_stamp = 0
        self.first_watchtower_message = 0
        self.last_odometry_time_stamp = 0
        self.cyclic_counter = CyclicCounter(100000, 1000, self.node_id)
        self.stocking_time = stocking_time
        self.result_folder = result_folder
        self.last_time_stamp = -1
        self.has_a_result_file = False
        self.had_odometry_before = False
        self.last_watchtower_time_stamp = -1

    def check_if_first_watchtower_msg(self, time_stamp):
        with self.node_lock:
            if self.first_watchtower_message == 0:
                self.first_watchtower_message = time_stamp

    def set_last_odometry_time_stamp(self, time_stamp):
        with self.node_lock:
            self.last_odometry_time_stamp = time_stamp

    def set_last_watchtower_time_stamp(self, time_stamp):
        with self.node_lock:
            if time_stamp > self.last_watchtower_time_stamp:
                self.last_watchtower_time_stamp = time_stamp

    def set_first_odometry_time_stamp(self, time_stamp):
        with self.node_lock:
            self.first_odometry_time_stamp = time_stamp
            self.had_odometry_before = True

    def is_movable(self):
        return True

    def has_had_odometry_before(self):
        return self.had_odometry_before

    def get_g2o_index(self, time_stamp):
        """Given a timestamp associated to that node, outputs an integer that can be used as a
           index for the node in g2o (the latter only handles integer indices
           for the nodes).

           Args:
              time_stamp: Timestamp.

           Returns:
              Input ID converted to an integer that can be used as an index by
              g2o.
        """
        if time_stamp == -1:
            return -1
        index = self.time_stamps_to_indices[time_stamp]
        return super(MovableNode, self).get_g2o_index(time_stamp) + index

    def add_timestamp(self, time_stamp):
        if (time_stamp not in self.time_stamps_to_indices):
            # Assign the next available local index to the timestamp and
            # increment the number of local indices assigned.
            with self.node_lock:
                self.time_stamps_to_indices[time_stamp] = self.cyclic_counter.next_index(
                )

            # If the new timestamp is posterior to all previously-received
            # timestamps simply set it to be the timestamp furthest in time
            # among those associated to the node. If, on the contrary, due to
            # delays in the transmission/reception of the messages this is not
            # the case, perform retro-interpolation if enabled.
            if (time_stamp > self.last_time_stamp):
                with self.node_lock:
                    self.last_time_stamp = time_stamp
            else:
                if self.retro_interpolate:
                    # Check that message is in the odometry chained part of the graph
                    if(time_stamp > self.first_odometry_time_stamp and time_stamp < self.last_odometry_time_stamp):
                        # check that optimization was made at least once, so that no absurd edge is created
                        if(self.duckietown_graph.chi2 != 0.0):
                            self.retrointerpolate(time_stamp)
                pass

            return True
        return False

    def interpolate(self, old_time_stamp, new_time_stamp,
                    measure, measure_information=None):
        """Given a timestamp new_time_stamp at which an odometry message was
           received and the timestamp old_time_stamp of the last odometry
           message previously received, it might be the case that other
           messages, of non-odometry type, have been received in the time
           interval between these two timestamps. For instance, a Duckiebot
           might be seen by a watchtower at two timestamps time_stamp1 and
           time_stamp2 s.t. old_time_stamp < time_stamp1 < time_stamp2 <
           new_time_stamp. This function creates edges in the graph between each
           pair of vertices at consecutive timestamps (e.g.
           old_time_stamp-->time_stamp1, time_stamp1-->time_stamp2,
           time_stamp2-->new_time_stamp). The relative transform between each
           pair of vertices is assigned by performing a linear interpolation in
           the Lie algebra, based on the transform between old_time_stamp and
           new_time_stamp (contained in the odometry message) and on the
           relative time difference (e.g. time_stamp2 - time_stamp1).

           Args:
               old_time_stamp: Timestamp of the last odometry message that was
                               received before the current odometry message, for
                               the node of type node_type and ID node_id.
               new_time_stamp: Timestamp of the current odometry message.
               node_type: Type of the node.
               node_id: ID of the node.
               measure: Transform contained in the current odometry message,
                        between timestamp old_time_stamp and time_stamp
                        new_time_stamp.
        """
        # Timestamps for which the interpolation should be performed. Note: also
        # old_time_stamp and new_time_stamp are included.
        with self.node_lock:
            to_interpolate = {
                time_stamp
                for time_stamp in self.time_stamps_to_indices
                if (time_stamp >= old_time_stamp and time_stamp <= new_time_stamp)
            }
        # Sort the time stamps.
        sorted_time_stamps = sorted(to_interpolate)
        # print("perfoming interpolation on %d nodes" % len(sorted_time_stamps))
        # Find the total time (time between the last and the first timestamp).
        total_delta_t = float(sorted_time_stamps[-1] - sorted_time_stamps[0])
        if (total_delta_t == 0.0):
            print(("in interpolate, delta t is 0.0, with %s and list is:" %
                  self.node_id))
            print(to_interpolate)
            print(("new_time_stamp is %f and old time stamp is %f" %
                  (old_time_stamp, new_time_stamp)))
            pprint.pprint(self.time_stamps_to_indices)
        # Perform a linear interpolation in the Lie algebra associated to SE3
        # group defined by the transform.

        cumulative_alpha = 0.0
        interpolation_list = []
        for i in range(0, len(sorted_time_stamps) - 1):
            # Find the time interval between each timestamp and the subsequent
            # one and linearly interpolate accordingly in the algebra.
            partial_delta_t = float(
                sorted_time_stamps[i + 1] - sorted_time_stamps[i])
            alpha = partial_delta_t / total_delta_t
            cumulative_alpha += alpha
            interpolated_measure = interpolate_measure(measure, alpha)

            vertex0_index = self.get_g2o_index(sorted_time_stamps[i])
            vertex1_index = self.get_g2o_index(sorted_time_stamps[i + 1])

            # Add an edge to the graph, connecting each timestamp to the one
            # following it and using the relative transform obtained by
            # interpolation.
            interpolation_list.append((vertex0_index, vertex1_index,
                                       interpolated_measure, 0.1, measure_information))
        if (cumulative_alpha != 1.0):
            pass
        for args in interpolation_list:
            self.duckietown_graph.graph.add_edge(*args)

    def retrointerpolate(self, time_stamp):
        # We cant have measure info for retro interpolation. We can create one that is strong, by assuming that all ready processed stuff is good.
        # We make orientation strong, distance weak
        space_dev = self.duckietown_graph.default_variance["retrointerpolation"]["position_deviation"]
        angle_dev = self.duckietown_graph.default_variance["retrointerpolation"]["angle_deviation"]
        retro_measure_information = create_info_matrix(space_dev, angle_dev)

        time_stamp_before = 0.0
        time_stamp_after = float('inf')
        with self.node_lock:
            for time_stamp_it in self.time_stamps_to_indices:
                if time_stamp_it < time_stamp:
                    if time_stamp_before < time_stamp_it:
                        time_stamp_before = time_stamp_it

                if time_stamp < time_stamp_it:
                    if time_stamp_it < time_stamp_after:
                        time_stamp_after = time_stamp_it

        # If the timestamp is neither the first nor the last (i.e., both
        # timestamp before and timestamp after exist) we can go ahead.
        if (time_stamp_before != 0.0 and time_stamp_after != float('inf')):
            before = self.get_g2o_index(time_stamp_before)
            after = self.get_g2o_index(time_stamp_after)
            transform = self.duckietown_graph.graph.get_transform(
                before, after)
            # If the timestamps before/after have corresponding vertices in the
            # graph perform retro-interpolation.
            if (transform != 0):
                # TODO : make a sanity check?
                self.interpolate(time_stamp_before,
                                 time_stamp_after, transform, measure_information=retro_measure_information)
            else:
                print(("will not perform retro_interpolation with %d and %d " %
                      (before, after)))

    def save_and_remove_old_poses(self, reference_time):
        last_accepted_stamps = reference_time - self.stocking_time
        max_anterior = None
        anterior_time_stamps_indices = []
        with self.node_lock:
            anterior_time_stamps = [time_stamp for time_stamp in list(self.time_stamps_to_indices.keys(
            )) if time_stamp < last_accepted_stamps]
        if(anterior_time_stamps != []):
            max_anterior = max(anterior_time_stamps)
            anterior_time_stamps.remove(max_anterior)
            # self.set_fixed(node_type, node_id, max_anterior)
            trajectory_list = self.extract_trajectory(max_anterior)
            if(trajectory_list is not None):
                self.save_trajectory(trajectory_list)
            elif trajectory_list == []:
                print("No previous trajectory")
                pass
            else:
                pass
        else:
            pass
        # Now that it is saved, remove it from the graph
        with self.node_lock:
            for time_stamp in anterior_time_stamps:
                self.cyclic_counter.remove_index(
                    self.time_stamps_to_indices[time_stamp])
                anterior_time_stamps_indices.append(
                    self.get_g2o_index(time_stamp))
                self.time_stamps_to_indices.pop(time_stamp)
            if(max_anterior != None):
                self.set_fixed(max_anterior)
        for index in anterior_time_stamps_indices:
            self.duckietown_graph.remove_vertex_by_index(index)

    def save_and_remove_everything(self, remove=False):
        time_stamps_indices = []
        with self.node_lock:
            time_stamps = list(self.time_stamps_to_indices.keys())
        if(time_stamps != []):
            pose_stamped_list = []
            for time_stamp in sorted(time_stamps):
                pose = self.duckietown_graph.get_vertex_pose(
                    self.get_g2o_index(time_stamp))
                pose_stamped = (time_stamp, pose)
                pose_stamped_list.append(pose_stamped)
            if(pose_stamped_list is not None):
                self.save_trajectory(pose_stamped_list)
            elif pose_stamped_list == []:
                print("No stored trajectory")
                pass
            else:
                pass
        else:
            pass
        # Now that it is saved, remove it from the graph
        if remove:
            with self.node_lock:
                for time_stamp in time_stamps:
                    self.cyclic_counter.remove_index(
                        self.time_stamps_to_indices[time_stamp])
                    time_stamps_indices.append(
                        self.get_g2o_index(time_stamp))
                    self.time_stamps_to_indices.pop(time_stamp)
            for index in time_stamps_indices:
                self.duckietown_graph.remove_vertex_by_index(index)

    def set_fixed(self, time_stamp):
        self.duckietown_graph.set_fixed(self.node_id, time_stamp)

    def extract_trajectory(self, target_time_stamp):
        """ This functions extracts the trajectory (which is a list(time stamps + pose))
            in a readable format, from oldest time_stamp to the given target_time_stamp

            returns : a list of tuples (time_stamp, pose)
                      poses are of g2o format Isometry3d(R, p)
        """
        with self.node_lock:
            time_stamps = sorted(self.time_stamps_to_indices.keys())
        if (target_time_stamp in time_stamps):
            target_index = time_stamps.index(target_time_stamp)
            time_stamps = time_stamps[:target_index]

            # Create list and retrieve all poses
            pose_stamped_list = []
            for time_stamp in sorted(time_stamps):
                pose = self.duckietown_graph.get_vertex_pose(
                    self.get_g2o_index(time_stamp))
                pose_stamped = (time_stamp, pose)
                pose_stamped_list.append(pose_stamped)
            return pose_stamped_list
        else:
            print(("[WARNING] trying to extract poses up until an unknown time stamp %f for node %s" % (
                target_time_stamp, self.node_id)))
            return None

    def save_trajectory(self, poses_stamped):
        """ Saves the trajectory to a file corresponding to node_type, node_id
            return True is success, False otherwise
        """
        result_file = "%s/%s.yaml" % (self.result_folder, self.node_id)
        if os.path.isdir(self.result_folder):
            if self.has_a_result_file:
                mode = 'a+'
            else:
                mode = 'w'
            with open(result_file, mode=mode) as trajectory_yaml:
                if not self.has_a_result_file:
                    header = ["x", "y", "z", "R11", "R12",
                              "R13", "R21", "R22", "R23", "R31", "R32", "R33"]
                    data = {"data_header": {"time_stamp": header}}
                    data["trajectory_data"] = {}
                else:
                    data = yaml.load(trajectory_yaml)

                for pose in poses_stamped:
                    row = []
                    for i in range(0, 3):
                        row.append(str("%.3f" % pose[1].t[i]))
                    for i in range(0, 3):
                        for k in range(0, 3):
                            row.append(str("%.5f" % pose[1].R[i][k]))

                    # print(row)
                    # row = ["coucou", 4, "les cococs"]

                    data["trajectory_data"][pose[0]] = row

                time_stamps = list(data["trajectory_data"].keys())
                min_time_stamp = min(time_stamps)
                data["begin_time_stamp"] = min_time_stamp

                keys = {}
                for key, _ in list(data["trajectory_data"].items()):
                    new_key = key - min_time_stamp
                    keys[key] = "%08.4f" % new_key

                for key, new_key in list(keys.items()):
                    data["trajectory_data"][new_key] = data["trajectory_data"].pop(
                        key)
                yaml.dump(data, trajectory_yaml)
                self.has_a_result_file = True

            # TODO : Code this function
            print(("Trying to save for %s" % self.node_id))
        else:
            print(("Couldnt save data for %s as result folder %s does not exist" % (
                self.node_id, self.result_folder)))

    def get_trajectory(self):
        result_dict = dict()
        g2o_vertices = self.duckietown_graph.graph.optimizer.vertices()
        with self.node_lock:
            for time_stamp, _ in list(self.time_stamps_to_indices.items()):
                vertex_id = self.get_g2o_index(time_stamp)
                if (vertex_id not in g2o_vertices):
                    pass
                else:
                    result_dict[time_stamp] = self.duckietown_graph.get_vertex_pose(
                        vertex_id)
        return result_dict

    def clean(self):
        anterior_time_stamps_indices = []

        for first_time_stamp in [self.first_odometry_time_stamp, self.first_watchtower_message]:
            if(first_time_stamp != 0):
                with self.node_lock:
                    anterior_time_stamps = [time_stamp for time_stamp in list(self.time_stamps_to_indices.keys(
                    )) if time_stamp <= first_time_stamp]
                    for time_stamp in anterior_time_stamps:
                        self.cyclic_counter.remove_index(
                            self.time_stamps_to_indices[time_stamp])
                        anterior_time_stamps_indices.append(
                            self.get_g2o_index(time_stamp))
                        self.time_stamps_to_indices.pop(time_stamp)
                for index in anterior_time_stamps_indices:
                    self.duckietown_graph.remove_vertex_by_index(index)

    def final_clean(self):
        self.clean()
        posterior_time_stamps_indices = []

        if(self.last_watchtower_time_stamp != -1):
            with self.node_lock:
                posterior_time_stamps = [time_stamp for time_stamp in list(self.time_stamps_to_indices.keys(
                )) if time_stamp > self.last_watchtower_time_stamp]
                for time_stamp in posterior_time_stamps:
                    self.cyclic_counter.remove_index(
                        self.time_stamps_to_indices[time_stamp])
                    posterior_time_stamps_indices.append(
                        self.get_g2o_index(time_stamp))
                    self.time_stamps_to_indices.pop(time_stamp)
            for index in posterior_time_stamps_indices:
                self.duckietown_graph.remove_vertex_by_index(index)


class DuckietownGraphBuilder(object):
    """Creates an internal g2o pose graph, and optimizes over it. At the same
       time, it keeps track of the timestamps at which messages related to an
       object (e.g. watchtower, Duckiebot) were received. We refer to objects as
       "nodes", with each node being identified by the ID of its April tag
       (node_id) and by its type (node_type). Types are defined in the class
       constructor and could e.g. be Duckiebots, watchtowers, etc.
       We instead refer to the nodes in the pose graph as "vertices".

       Attributes:
           graph: Pose graph (g2oGraphBuilder object).
           types: Possible types of nodes in the graph, i.e., of objects in
                  Duckietown.
           movable: List of the types of nodes that, for the purposes of
                    optimization, should be considered as objects that can move
                    in Duckietown (pose can be optimized).
           cyclic_counters: A custom class of cyclic counters that will give indices
                            and cycle back to 0 as the first used indices are freed
                            by cleaning procedures
           last_time_stamp: Dictionary of dictionaries.
                            last_time_stamp[<node_type>][<node_id>] contains the
                            timestamp latest in time among those associated to
                            the node of type <node_type> and with ID <node_id>.
           last/first_odometry_time_stamp:
                    Dictionaries of dictionaries.
                    last/first_odometry_time_stamp[<node_type>][<node_id>]
                    contains the timestamp at which the last/first odometry
                    message for the node with type <node_type> and ID <node_id>
                    was received. This information is kept track of to ensure
                    that retro-interpolation is only performed between the
                    first and the last odometry message.
           retro_interpolate: True if retro-interpolation should be performed,
                              False otherwise. Retro-interpolation allows to
                              handle messages with a timestamp anterior to the
                              latest timestamp of the already-processed
                              messages. See retrointerpolate method for further
                              details.
           chi2 : The measure of the uncertainty of the graph, given by the optimizer in g2o
    """

    ###################################
    #       BUILDING FUNCTIONS        #
    ###################################

    def __init__(self,
                 initial_duckiebot_dict={},
                 initial_watchtower_dict={},
                 initial_april_dict={},
                 initial_floor_april_tags="",
                 retro_interpolate=True,
                 using_priors=True,
                 stocking_time=None,
                 priors_filename=None,
                 default_variance=None,
                 result_folder="/tmp"):
        # Initialize pose graph.
        self.graph = g2oGB.g2oGraphBuilder()
        # Define node types.
        self.types = ["duckiebot", "watchtower", "apriltag"]

        # Define movable node types.
        self.movable = ["duckiebot"]
        # Initialize chi2
        self.chi2 = 0.0
        # Set retro-interpolate mode as inputted.
        self.retro_interpolate = retro_interpolate
       # Load the initial floor April tags if given an input file name
        self.lock = g2oGB.ControlableLock()

        self.node_dict = dict()

        self.result_folder = result_folder

        self.stocking_time = stocking_time
        self.last_cleaning = 0.0
        self.timeout = 5.0
        self.using_priors = using_priors
        self.default_variance = default_variance
        self.last_callback = -1
        if(priors_filename is not None):
            self.priors = PriorHandler(priors_filename)

        if (initial_floor_april_tags != ""):
            self.load_initial_floor_april_tags(initial_floor_april_tags)
        print("init done")

    def wait(self):
        now = time.time()
        if self.last_callback == -1:
            self.last_callback = now
        while now - self.last_callback <= self.timeout:
            time.sleep(1.0)
            now = time.time()

    def load_initial_floor_april_tags(self, initial_floor_april_tag_file):
        """Adds the poses of the initial floor April tags to the graph by
           reading them from file.

           Args:
               initial_floor_april_tag_file: File from where to read the poses
                                             of the floor tags.
        """
        # TODO: Use duckietown-world to load the tags.
        with open(initial_floor_april_tag_file, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                # Add a reference vertex to the g2o graph.
                vertex_pose = g2o.Isometry3d(np.eye(3), [0, 0, 0])
                # with self.lock:
                self.graph.add_vertex(0, vertex_pose, fixed=True)
                # Add vertices for all the floor tags.
                for key, value in list(complete_dict.items()):
                    if key == "objects":
                        for _, object_value in list(value.items()):
                            if (object_value['kind'] == "floor_tag"):
                                tag_id = object_value['tag']['~TagInstance'][
                                    'tag_id']
                                position = object_value['pose'][
                                    '~SE2Transform']['p']
                                theta = 0
                                if ("theta_deg" in object_value['pose'][
                                        '~SE2Transform']):
                                    theta = object_value['pose'][
                                        '~SE2Transform']['theta_deg']
                                vertex_id = "apriltag_%d" % tag_id
                                # with self.lock:
                                self.add_vertex(
                                    vertex_id,
                                    theta,
                                    position,
                                    is_initial_floor_tag=True,
                                    fixed=True)
            except yaml.YAMLError as exc:
                print(exc)

    def get_node(self, node_id):
        if(node_id in self.node_dict):
            return self.node_dict[node_id]
        else:
            with self.lock:
                node_type = get_node_type(node_id)
                if (node_type in self.movable):
                    node = MovableNode(node_id, self.types,
                                       self, stocking_time=self.stocking_time, retro_interpolate=self.retro_interpolate, result_folder=self.result_folder)
                    self.node_dict[node_id] = node

                else:
                    node = Node(node_id, self.types, self)
                    self.node_dict[node_id] = node
                return node

    def add_timestamp_to_node(self, node_id, time_stamp):
        """TODO : all docs!!

           Args:
               node_id: ID of the node to which the timestamp should be
                        associated.
               time_stamp: Timestamp.

           Returns:
               True if the operation is successful, False otherwise (i.e., if
               the timestamp is already associated to the node with a 'local
               index' or if the the node is not movable and already has
               an associated timestamp).
        """
        node = self.get_node(node_id)

        return node.add_timestamp(time_stamp)

    def add_vertex(self,
                   node_id,
                   theta,
                   position,
                   is_initial_floor_tag=False,
                   fixed=False,
                   time_stamp=0.0):
        """Adds a node with an ID given in the format outputted from the
          transform_listener and with a given pose to the g2o graph.

           TODO : add a more general add_vertex function that takes a 3D pose and not only a 2D one.

           Args:
               node_id: ID of the node whose pose should be added as a node
                          in the graph, in the format <node_type>_<node_id>.
               theta: Angle of the 2D rotation around the z axis defines the
                      pose of the node.
               position: 2D translation vector (x, y) that define the pose of the node.
               is_initial_floor_tag: True if the node is an initial floor tag,
                                     False otherwise.
               fixed: True if the node should be added as a fixed node (i.e.,
                      not optimizable) in the graph, False otherwise.
               time_stamp: Timestamp.
        """
        # Adds (assigns) a timestamp to a node, by giving it a 'local index'
        # w.r.t. the node.
        node = self.get_node(node_id)
        added = node.add_timestamp(time_stamp)
        if (not added):
            print((
                "add_vertex did not add : node %s at time %f as already there" %
                (node_id, time_stamp)))
        else:
            # Translation vector, z coordinate does not vary.
            position = [position[0], position[1], 0.0]
            # Rotation around the z axis of and angle theta.
            R = g.rotation_from_axis_angle(
                np.array([0, 0, 1]), np.deg2rad(theta))
            if (is_initial_floor_tag):
                # For initial floor tags, apply a further rotation of 180
                # degrees around the x axis, so that the orientation of the z
                # axis is reverted.
                R2 = g.rotation_from_axis_angle(
                    np.array([1, 0, 0]), np.deg2rad(180))
                R = np.matmul(R, R2)
            # Add vertex with pose and ID in the right format to the g2o graph.
            vertex_pose = g2o.Isometry3d(R, position)
            vertex_index = node.get_g2o_index(time_stamp)
            self.graph.add_vertex(vertex_index, vertex_pose, fixed=fixed)

            if (is_initial_floor_tag):
                # For initial floor tags, add an edge between the reference
                # vertex and the newly-added vertex for the pose of the tag.
                self.graph.add_edge(
                    vertex0_id=0, vertex1_id=vertex_index, measure=vertex_pose, measure_information=None)

    def add_edge(self,
                 node_id_0,
                 node_id_1,
                 measure,
                 time_stamp,
                 old_time_stamp=0,
                 measure_information=None):
        """Adds edge between two vertices to the graph.

           Args:
               vertex0/1_id: IDs of the vertices associated to the edge that
                             should be added. For instance the vertices might
                             correspond to the poses of a watchtower and of a
                             tag seen by the watchtower.
               measure: Transform ('measurement' in g2o) associated to the
                        message (and therefore to the edge added to the graph).
               time_stamp: Timestamp of the message that is associated to the
                           edge added to the graph.
               old_time_stamp: Needed for odometry messages. Contains the
                               timestamp of the last odometry message previously
                               received by the object with the given ID (which
                               for odometry messages is either vertex0_id or
                               vertex1_id, since the two match).
        """
        if (node_id_0 != node_id_1):
            node_0 = self.get_node(node_id_0)
            node_1 = self.get_node(node_id_1)
            # with self.lock:
            node_0.add_timestamp(time_stamp)
            node_1.add_timestamp(time_stamp)
            if(node_0.node_type == "watchtower" and node_1.is_movable()):
                node_1.check_if_first_watchtower_msg(time_stamp)
                node_1.set_last_watchtower_time_stamp(time_stamp)
            # Obtain ID of the vertices in the graph in the integer format.
            vertex0_index = node_0.get_g2o_index(time_stamp)
            vertex1_index = node_1.get_g2o_index(time_stamp)
            # Add edge between the two vertices (and automatically also add the
            # vertices if not there already).in the g2o graph.
            self.graph.add_edge(vertex0_index, vertex1_index,
                                measure, robust_kernel_value=0.1, measure_information=measure_information)
            if self.using_priors:
                self.add_priors(node_id_0, node_id_1, time_stamp)
            self.last_callback = time.time()
        else:
            node = self.get_node(node_id_0)
            # Associate timestamps to the vertex.
            # with self.lock:
            node.add_timestamp(time_stamp)

            if node.is_movable():
                # Odometry edge: same vertices, movable object.
                if not node.has_had_odometry_before():
                    # No previous odometry messages -> Update old timestamp for
                    # the node to be the oldest timestamp received..

                    # Initialize the first and last timestamp at which an odometry message for
                    # the node was received to be respectively old_time_stamp and the current
                    # timestamp (time_stamp).
                    node.set_first_odometry_time_stamp(time_stamp)
                    node.set_last_odometry_time_stamp(time_stamp)
                    # Add vertex corresponding to the new timestamp to the
                    # graph.
                    self.add_vertex(
                        node_id=node_id_0,
                        theta=0,
                        position=[0, 0],
                        is_initial_floor_tag=False,
                        fixed=False,
                        time_stamp=time_stamp)
                    self.last_callback = time.time()

                else:
                    # Update the known last odometry message time_stamp for the node
                    node.set_last_odometry_time_stamp(time_stamp)

                    # Some other messages might have been received for that node
                    # (object), e.g. a Duckiebot might be seen by a watchtower
                    # before receiving an odometry message => Interpolate.
                    node.interpolate(old_time_stamp, time_stamp,
                                     measure, measure_information=measure_information)

                    self.last_callback = time.time()
                    return True

            else:
                print((
                    "Node type %s should be movable if given odometry transform"
                    % node_id_0))
                exit(-1)

    def set_fixed(self, node_id, time_stamp):
        """ Setting to fixed a specific node
        """
        node = self.get_node(node_id)
        self.graph.set_fixed(node.get_g2o_index(time_stamp))

    def add_priors(self, node_id_0, node_id_1, time_stamp):
        node0 = self.get_node(node_id_0)
        node1 = self.get_node(node_id_1)

        for prior in self.priors.prior_list:
            if(prior.is_from_origin()):
                for node in [node0, node1]:
                    if(prior.unary_concerns(node.node_type)):
                        # print("adding prior from origin")
                        self.graph.add_edge(0, node.get_g2o_index(
                            time_stamp), prior.measure, robust_kernel_value=0.1, measure_information=prior.measure_information)
            else:
                if prior.binary_concerns(node0.node_type, node1.node_type):
                    self.graph.add_edge(node0.get_g2o_index(time_stamp), node1.get_g2o_index(
                        time_stamp), prior.measure, robust_kernel_value=0.1, measure_information=prior.measure_information)

    ###################################
    #        OPTIM FUNCTIONS          #
    ###################################

    def get_global_last_time(self):
        global_last_time = 0
        with self.lock:
            for _, node in list(self.node_dict.items()):
                if node.last_time_stamp > global_last_time:
                    global_last_time = node.last_time_stamp
        return global_last_time

    def optimize(self,
                 number_of_steps,
                 verbose=True,
                 save_result=True,
                 output_name="output.g2o",
                 online=False,
                 final=False):
        """Performs optimization.

            Args:
                number_of_steps: Number of steps of the optimization performed
                                 by g2o.
                verbose: Set to true for detailed prints.
                save_result: Set to true to save the result of the optimization
                             to file.
                output_name: Output filename of the result of the optimization.
         """
        # TODO : cleaning is gonna be useless when remove_old_poses will have run many times
        # Maybe find a way to stop doing it after a while
        if final:
            self.final_clean_graph()
        else:
            self.clean_graph()
        if self.stocking_time is not None:
            global_last_time_stamp = self.get_global_last_time()
            if(global_last_time_stamp - self.last_cleaning > self.stocking_time/2.0):
                self.save_and_remove_old_poses(global_last_time_stamp)
                self.last_cleaning = global_last_time_stamp

        self.chi2 = self.graph.optimize(
            number_of_steps,
            verbose=verbose,
            save_result=save_result,
            output_name=output_name,
            online=online)

    ###################################
    #       CLEANING FUNCTIONS        #
    ###################################

    def remove_vertex(self, node_id, time_stamp):
        node = self.get_node(node_id)
        self.graph.remove_vertex(node.get_g2o_index(time_stamp))

    def remove_vertex_by_index(self, index):
        self.graph.remove_vertex(index)

    def save_and_remove_old_poses(self, reference_time):
        """
            Gets rid of old vertices in the graph
            TODO : register in a file the path of duckiebots before destroying it here
        """
        with self.lock:
            for _, node in list(self.node_dict.items()):
                if node.is_movable():
                    node.save_and_remove_old_poses(reference_time)

    def clean_graph(self):
        """
            Gets rid of useless vertices in the graph
            Considered as useless are vertices that are anterior to the first odometry message
        """
        with self.lock:
            for _, node in list(self.node_dict.items()):
                if node.is_movable():
                    node.clean()

    def final_clean_graph(self):
        """
            Gets rid of useless vertices in the graph
            Considered as useless are vertices that are anterior to the first odometry message
        """
        with self.lock:
            for _, node in list(self.node_dict.items()):
                if node.is_movable():
                    node.final_clean()

    ###################################
    #        ACCESS FUNCTIONS         #
    ###################################

    def get_vertex_pose(self, vertex_id):
        return self.graph.vertex_pose(vertex_id)

    def get_all_poses(self):
        """Obtains all poses in the graph.

           Returns:
               Dictionary of dictionaries, containing for each type of node and
               each node ID the pose corresponding to the latest timestamp
               available in the graph. For instance,
               result_dict[node_type][node_id] contains the latest pose
               available in the g2o graph for the node with type <node_type> and
               ID <node_id>.
        """
        with self.lock:
            result_dict = dict()
            # TODO : dictionnary changed size during iteration!! WTF
            for node_id, node in list(self.node_dict.items()):
                result_dict[node_id] = node.get_last_known_position()

        return result_dict

    def get_movable_paths(self):
        """Obtains the history of poses for each movable object.

           Returns:
               Dictionary of dictionaries, containing for each type of movable node and
               each node ID the history of poses
               available in the graph. For instance,
               result_dict[node_type][node_id] contains all the poses
               in the g2o graph for the node with type <node_type> and
               ID <node_id>.
        """
        result_dict = dict()
        with self.lock:
            for node_id, node in list(self.node_dict.items()):
                if node.is_movable():
                    result_dict[node_id] = node.get_trajectory()

        return result_dict

    def on_shutdown(self):
        with self.lock:
            for node in list(self.node_dict.values()):
                if node.is_movable():
                    node.save_and_remove_everything()
        print("exited duckietown graph builder")
