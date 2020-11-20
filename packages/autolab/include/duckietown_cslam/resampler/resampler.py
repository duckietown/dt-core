import duckietown_cslam.duckietownGraphBuilder.duckietown_graph_builder as dGB

import random
import threading
import csv
import yaml
import os
import time

import g2o
import geometry as g
import numpy as np


def merge_measure_information(list_measure):
    result = np.zeros((6, 6))

    if len(list_measure) == 0:
        return None
    for measure_info in list_measure:
        result += np.linalg.inv(measure_info)
    result /= len(list_measure)
    result = np.linalg.inv(result)
    return result


def get_closest_rounded(seed, time_stamp, interval):
    if(seed == -1):
        print("not normal")
    while time_stamp - seed > interval:
        seed += interval
    return seed


def interpolate_measure(measure, alpha):
    R = measure.R
    t = measure.t
    q = g.SE3_from_rotation_translation(R, t)
    vel = g.SE3.algebra_from_group(q)

    rel = g.SE3.group_from_algebra(vel * alpha)
    newR, newt = g.rotation_translation_from_SE3(rel)
    return g2o.Isometry3d(newR, newt)


class OdometryResampler(object):
    class MyThread(threading.Thread):
        def __init__(self, event, function, time_interval):
            threading.Thread.__init__(self)
            self.stopped = event
            self.function = function
            self.time_interval = time_interval

        def run(self):
            while not self.stopped.wait(self.time_interval):
                # print("my thread")
                self.function()

    def __init__(self, duckiebot_id, pose_graph, first_time_stamp, reference_time_stamp, frequency):
        self.id = duckiebot_id
        self.edges = {}
        self.last_time_stamps = []
        self.max_time_diff = 0.1
        self.last_odometry_time_stamp = 0
        self.pose_graph = pose_graph
        self.frequency = frequency
        self.time_interval = 1.0 / self.frequency
        self.first_time_stamp = get_closest_rounded(
            reference_time_stamp, first_time_stamp, self.time_interval)
        self.last_received_time_stamp = first_time_stamp
        self.stopFlag = threading.Event()

        self.resampler_recall = self.MyThread(
            self.stopFlag, self.resampler_callback, self.time_interval)
        self.last_sent_time_stamp = self.first_time_stamp
        self.resampler_recall.start()

    def add_edge(self,
                 measure,
                 time_stamp,
                 measure_information=None):
        self.edges[time_stamp] = (measure, measure_information)
        if time_stamp > self.last_odometry_time_stamp:
            self.last_time_stamps.append(time_stamp)
            self.last_time_stamps.sort()
        if time_stamp > self.last_received_time_stamp:
            self.last_received_time_stamp = time_stamp

    def resampler_callback(self):
        while self.last_received_time_stamp - self.last_sent_time_stamp > self.time_interval:
            self.last_sent_time_stamp += self.time_interval
            transform, old_time_stamp, measure_information = self.get_transform(
                self.last_sent_time_stamp)
            if transform != -1:
                self.pose_graph.add_edge(
                    self.id, self.id, transform, self.last_sent_time_stamp, old_time_stamp=old_time_stamp, measure_information=measure_information)

    def get_transform(self, time_stamp):
        if self.last_odometry_time_stamp == 0:
            self.last_odometry_time_stamp = time_stamp
            self.last_time_stamps = [x for x in self.last_time_stamps if x >= time_stamp - self.max_time_diff]
            print("First odometry message")
            return -1, -1, -1
        previous_time_stamp = 0
        next_time_stamp = 10**10
        time_stamp_to_use = []
        list_measure_info = []
        if len(self.last_time_stamps) > 0 and self.last_odometry_time_stamp > min(self.last_time_stamps):
            min_last_time_stamps = min(self.last_time_stamps)
            max_last_time_stamps = max(self.last_time_stamps)

            if time_stamp > max_last_time_stamps:
                print(("asking for too recent time stamp : %f is bigger than last %f" % (
                    time_stamp, max_last_time_stamps)))
                self.last_odometry_time_stamp = time_stamp
                return -1, -1, -1

            if self.last_odometry_time_stamp >= min_last_time_stamps:

                for t in self.last_time_stamps:
                    if t < self.last_odometry_time_stamp and t > previous_time_stamp:
                        previous_time_stamp = t
                    if t > time_stamp and t < next_time_stamp:
                        next_time_stamp = t
                    if t <= time_stamp and t >= self.last_odometry_time_stamp:
                        time_stamp_to_use.append(t)
                        list_measure_info.append(
                            self.edges[t][1])
        else:
            for t in list(self.edges.keys()):
                if t < self.last_odometry_time_stamp and t > previous_time_stamp:
                    previous_time_stamp = t
                if t > time_stamp and t < next_time_stamp:
                    next_time_stamp = t
                if t <= time_stamp and t >= self.last_odometry_time_stamp:
                    time_stamp_to_use.append(t)
                    list_measure_info.append(
                        self.edges[t][1])
        if previous_time_stamp == 0.0 or next_time_stamp == 10**10:
            # if next_time_stamp - previous_time_stamp > 2 * self.max_time_diff:
            print("too much time difference")
            self.last_odometry_time_stamp = time_stamp
            return -1, -1, -1

        time_stamp_to_use.sort()

        eye = np.eye(3)
        zero = np.array([0, 0, 0])
        middle_transform = g2o.Isometry3d(eye, zero)

        for i in range(len(time_stamp_to_use) - 1):
            middle_transform = middle_transform * \
                self.edges[time_stamp_to_use[i]][0]

        # if not (previous_time_stamp < self.last_odometry_time_stamp and self.last_odometry_time_stamp < min(time_stamp_to_use)):
        #     print("In odometry resampler, should not happen")

        # calculating the first interpolated transform
        if len(time_stamp_to_use) > 0:
            alpha = (min(time_stamp_to_use) - self.last_odometry_time_stamp) / \
                (min(time_stamp_to_use) - previous_time_stamp)

            previous_transform = self.edges[previous_time_stamp][0]

            first_transform_part = interpolate_measure(
                previous_transform, alpha)

            # calculating the last transform
            max_time_stamp = max(time_stamp_to_use)
            alpha = (time_stamp - max_time_stamp) / \
                (next_time_stamp - max_time_stamp)

            last_transform = self.edges[max_time_stamp][0]

            last_transform_part = interpolate_measure(last_transform, alpha)

            transform = first_transform_part * middle_transform * last_transform_part
            list_measure_info.append(
                self.edges[max_time_stamp][1])
            list_measure_info.append(
                self.edges[previous_time_stamp][1])
        else:
            alpha = (time_stamp - self.last_odometry_time_stamp) / \
                (next_time_stamp - previous_time_stamp)
            previous_transform = self.edges[previous_time_stamp][0]
            transform = interpolate_measure(previous_transform, alpha)
            max_time_stamp = previous_time_stamp
            list_measure_info.append(
                self.edges[previous_time_stamp][1])

        old_time_stamp = self.last_odometry_time_stamp
        self.last_odometry_time_stamp = time_stamp
        self.last_time_stamps = [x for x in self.last_time_stamps if x >= max_time_stamp]

        measure_information = merge_measure_information(list_measure_info)

        return (transform, old_time_stamp, measure_information)


class SingleDuckiebotTrajectory():
    class MyThread(threading.Thread):
        def __init__(self, event, function, time_interval):
            threading.Thread.__init__(self)
            self.stopped = event
            self.function = function
            self.time_interval = time_interval

        def run(self):
            while not self.stopped.wait(self.time_interval):
                # print("my thread")
                self.function()

    def __init__(self, duckiebot_id, first_time_stamp, watchtower_id, pose_graph, reference_time_stamp, frequency):
        self.id = duckiebot_id
        self.edges = {}
        self.last_time_stamps = []
        self.max_time_diff = 0.2
        self.watchtower_id = watchtower_id
        self.pose_graph = pose_graph
        self.frequency = frequency
        self.time_interval = 1.0 / self.frequency
        self.first_time_stamp = get_closest_rounded(
            reference_time_stamp, first_time_stamp, self.time_interval)
        self.last_received_time_stamp = first_time_stamp
        self.stopFlag = threading.Event()

        self.resampler_recall = self.MyThread(
            self.stopFlag, self.resampler_callback, self.time_interval)
        self.last_sent_time_stamp = self.first_time_stamp
        self.resampler_recall.start()

    def add_edge(self, measure, time_stamp, measure_information):
        self.edges[time_stamp] = (measure, measure_information)
        if self.last_time_stamps == [] or time_stamp > min(self.last_time_stamps):
            self.last_time_stamps.append(time_stamp)
            self.last_time_stamps.sort()
            if len(self.last_time_stamps) > 30:
                self.last_time_stamps.remove(min(self.last_time_stamps))
        if time_stamp > self.last_received_time_stamp:
            self.last_received_time_stamp = time_stamp

    def resampler_callback(self):
        while self.last_received_time_stamp - self.last_sent_time_stamp > self.time_interval:
            self.last_sent_time_stamp += self.time_interval
            transform, measure_information = self.get_transform(
                self.last_sent_time_stamp)
            if transform != -1:
                self.pose_graph.add_edge(
                    self.watchtower_id, self.id, transform, self.last_sent_time_stamp, measure_information=measure_information)

    def get_transform(self, time_stamp):
        previous_time_stamp = 0
        next_time_stamp = 10**10
        if len(self.last_time_stamps) > 0:
            min_last_time_stamps = min(self.last_time_stamps)
            max_last_time_stamps = max(self.last_time_stamps)

            if time_stamp > max_last_time_stamps:
                # print("Can not interpolate since no next time stamp.")
                return -1, -1

            if time_stamp > min_last_time_stamps:

                for t in self.last_time_stamps:
                    if t < time_stamp and t > previous_time_stamp:
                        previous_time_stamp = t
                    if t > time_stamp and t < next_time_stamp:
                        next_time_stamp = t
        else:
            for t in list(self.edges.keys()):
                if t < time_stamp and t > previous_time_stamp:
                    previous_time_stamp = t
                if t > time_stamp and t < next_time_stamp:
                    next_time_stamp = t

        if next_time_stamp - previous_time_stamp > self.max_time_diff:
            # We wont interpolate between far away points.
            return -1, -1

        p0, p0_info = self.edges[previous_time_stamp]
        p1, p1_info = self.edges[next_time_stamp]
        delta_p = p0.inverse() * p1

        total_delta_t = float(next_time_stamp - previous_time_stamp)
        delta_t = float(time_stamp - previous_time_stamp)

        alpha = delta_t / total_delta_t

        interpolated_measure = interpolate_measure(delta_p, alpha)

        final_transform = p0 * interpolated_measure

        list_measure = [p0_info, p1_info]
        measure_information = merge_measure_information(list_measure)

        return final_transform, measure_information


class WatchtowerTrajectroyResampler(object):
    def __init__(self, watchtower_id, pose_graph, reference_time_stamp, frequency):
        self.id = watchtower_id
        self.duckiebot_trajectories = {}
        self.pose_graph = pose_graph
        self.reference_time_stamp = reference_time_stamp
        self.frequency = frequency

    def add_edge(self,
                 duckiebot_id,
                 measure,
                 time_stamp,
                 measure_information=None):

        if duckiebot_id not in self.duckiebot_trajectories:
            duckiebot_trajectory = SingleDuckiebotTrajectory(
                duckiebot_id, time_stamp, self.id, self.pose_graph, self.reference_time_stamp, self.frequency)
            self.duckiebot_trajectories[duckiebot_id] = duckiebot_trajectory
        else:
            duckiebot_trajectory = self.duckiebot_trajectories[duckiebot_id]

        duckiebot_trajectory.add_edge(measure, time_stamp, measure_information)

    # def get_transform(self, duckiebot_id, time_stamp):
    #     if duckiebot_id not in self.duckiebot_trajectories:
    #         print("Error : no %s seen by %s" % (duckiebot_id, self.id))
    #         return -1, -1
    #     return self.duckiebot_trajectories[duckiebot_id].get_transform(time_stamp)

    def get_all_transforms(self, time_stamp):
        returnlist = []
        for duckiebot_id in self.duckiebot_trajectories:
            transform = self.duckiebot_trajectories[duckiebot_id].get_transform(
                time_stamp)
            if transform != -1:
                returnlist.append(
                    (self.id, duckiebot_id, transform, time_stamp))
        return returnlist


class Resampler():
    def __init__(self, initial_floor_april_tags, stocking_time, priors_filename, default_variance, using_priors, result_folder, resampling_frequency=20.0):
        self.pose_graph = dGB.DuckietownGraphBuilder(
            initial_floor_april_tags=initial_floor_april_tags, stocking_time=stocking_time, priors_filename=priors_filename, default_variance=default_variance, using_priors=using_priors, result_folder=result_folder)
        self.reference_time_stamp = -1
        self.watchtower_samplers = {}
        self.odometry_samplers = {}
        self.stopFlag = threading.Event()
        self.frequency = resampling_frequency
        self.time_interval = 1.0 / self.frequency
        self.last_callback = -1
        self.timeout = 5.0

    def handle_odometry_edge(self, duckiebot_id, measure, time_stamp, measure_information=None):
        if duckiebot_id not in self.odometry_samplers:
            self.odometry_samplers[duckiebot_id] = OdometryResampler(
                duckiebot_id, self.pose_graph, time_stamp, self.reference_time_stamp, self.frequency)
        odometry_sampler = self.odometry_samplers[duckiebot_id]
        odometry_sampler.add_edge(measure, time_stamp, measure_information)
        self.last_callback = time.time()

    def handle_watchtower_edge(self, watchtower_id, node_id1, measure, time_stamp, measure_information=None, is_duckiebot=False):
        if is_duckiebot:
            if watchtower_id not in self.watchtower_samplers:
                self.watchtower_samplers[watchtower_id] = WatchtowerTrajectroyResampler(
                    watchtower_id, self.pose_graph, self.reference_time_stamp, self.frequency)
            watchtower_trajectroy_resampler = self.watchtower_samplers[watchtower_id]
            watchtower_trajectroy_resampler.add_edge(
                node_id1, measure, time_stamp, measure_information)
        else:
            self.pose_graph.add_edge(
                watchtower_id, node_id1, measure, time_stamp, measure_information)
        self.last_callback = time.time()

    def handle_duckiebot_edge(self, duckiebot_id, node_id1, measure, time_stamp, measure_information=None):
        self.pose_graph.add_edge(
            duckiebot_id, node_id1, measure, time_stamp, measure_information)
        self.last_callback = time.time()

    def optimize(self,
                 max_iteration,
                 save_result,
                 verbose,
                 output_name,
                 online,
                 final=False):
        self.pose_graph.optimize(max_iteration, save_result=save_result,
                                 verbose=verbose, output_name=output_name, online=online, final=final)

    def get_all_optimized_poses(self):
        return self.pose_graph.get_all_poses()

    def get_optimized_movable_paths(self):
        return self.pose_graph.get_movable_paths()

    def wait(self):
        now = time.time()
        if self.last_callback == -1:
            self.last_callback = now
        while now - self.last_callback <= self.timeout:
            time.sleep(1.0)
            now = time.time()
        self.pose_graph.wait()

    def on_shutdown(self):
        self.stopFlag.set()
        print("Stopping resampling callback")
        self.pose_graph.on_shutdown()
        print("exiting resampler")

    def signal_reference_time_stamp(self, reference_time_stamp):
        self.reference_time_stamp = float(
            int(reference_time_stamp * self.frequency) / self.frequency)
