#!/usr/bin/env python
import rospy
import numpy as np
from matplotlib import pyplot as plt
from collections import deque
import time
from duckietown_msgs.msg import Twist2DStamped


class VehicleCommand(object):
    '''class for storing vehicle commands'''

    def __init__(self, v, omega, time):
        self.v = v
        self.omega = omega
        self.time = time


class PoseEstimator(object):
    '''class for estimating pose of Duckiebot at intersection'''

    def __init__(self):
        # estimator state
        # state = (x,y,theta)
        self.state_est = np.zeros(shape=3, dtype=float)
        self.cov_est = np.zeros(shape=(3, 3), dtype=float)
        self.time_est = rospy.Time.now()

        # vehicle command queue
        # inputs = (v,omega)
        self.cmd_queue = deque([VehicleCommand(0.0, 0.0, -1.0)])

        # estimator parameters
        self.cov_est_init = np.diag([1.0, 1.0, 0.5])
        self.cov_proc = np.diag([10, 50])  # process noise is assumed to be on inputs

        self.blocked = False

    def Reset(self, pose_init, time_init):
        '''reset state estimate'''
        self.state_est[:] = pose_init
        self.time_est = time_init

        self.cov_est[:] = self.cov_est_init

        self.cmd_queue = deque([VehicleCommand(0.0, 0.0, time_init)])

    def PredictState(self, time_pred, predict_cov=False):
        '''predict estimate forward until time_pred'''
        while self.blocked:
            time.sleep(0.00001)

        self.blocked = True

        state_est = np.copy(self.state_est)
        cov_est = np.copy(self.cov_est)
        time_est = self.time_est

        # integrate forward with vehicle commands
        idx_cmd = 0
        num_cmd = len(self.cmd_queue)
        while time_est < time_pred:
            # find current command
            if idx_cmd + 1 < num_cmd:
                dt = min(self.cmd_queue[idx_cmd + 1].time,
                         time_pred) - time_est
                dt_sec = dt.to_sec()  # careful, this could eventually cause problems if running long
            else:
                dt = time_pred - time_est
                dt_sec = dt.to_sec()

            # predict covariance
            if predict_cov:
                A = np.array([[0.0, 0.0, -self.cmd_queue[idx_cmd].v * np.sin(state_est[2])],
                              [0.0, 0.0, self.cmd_queue[idx_cmd].v * np.cos(state_est[2])],
                              [0.0, 0.0, 0.0]])
                L = np.array([[np.cos(state_est[2]), 0.0],
                              [np.sin(state_est[2]), 0.0],
                              [0.0, 1.0]])
                cov_dot = np.dot(A, cov_est) + np.dot(cov_est, A.T) + np.dot(L, np.dot(self.cov_proc, L.T))
                cov_est = cov_est + cov_dot * dt_sec

            # predict state
            if np.abs(self.cmd_queue[idx_cmd].omega) > 1e-6:
                radius = self.cmd_queue[idx_cmd].v / self.cmd_queue[idx_cmd].omega

                state_est[0] = (state_est[0] - radius * np.sin(state_est[2])) + radius * np.sin(
                    state_est[2] + self.cmd_queue[idx_cmd].omega * dt_sec)
                state_est[1] = (state_est[1] + radius * np.cos(state_est[2])) - radius * np.cos(
                    state_est[2] + self.cmd_queue[idx_cmd].omega * dt_sec)
            else:
                state_est[0] = state_est[0] + self.cmd_queue[idx_cmd].v * np.cos(state_est[2]) * dt_sec
                state_est[1] = state_est[1] + self.cmd_queue[idx_cmd].v * np.sin(state_est[2]) * dt_sec
            state_est[2] = state_est[2] + self.cmd_queue[idx_cmd].omega * dt_sec

            time_est = time_est + dt
            idx_cmd += 1

        self.blocked = False

        # make sure covariance matrix is symmetric
        cov_est = 0.5 * (cov_est + cov_est.T)

        return state_est, cov_est

    def FeedCommandQueue(self, msg):
        '''store applied commands in command queue'''
        self.cmd_queue.append(VehicleCommand(msg.v,msg.omega,msg.header.stamp))


    def UpdateWithPoseMeasurement(self, pose_meas, cov_meas, time_meas):
        '''update state estimate with pose measurement'''

        # prior update
        state_prior, cov_prior = self.PredictState(time_meas, True)

        while self.blocked:
            time.sleep(0.00001)

        self.blocked = True

        # remove old commands from queue
        while self.cmd_queue[0].time < time_meas and len(self.cmd_queue) > 1:
            self.cmd_queue.popleft()

        # a posteriori update
        K = np.dot(cov_prior, np.linalg.inv(cov_prior + cov_meas))
        state_posterior = state_prior + np.dot(K, pose_meas - state_prior)
        cov_posterior = cov_prior - np.dot(K, cov_prior)

        # store results
        self.state_est[:] = state_posterior
        self.cov_est[:] = cov_posterior
        self.time_est = time_meas

        self.blocked = False


if __name__ == '__main__':
    # debugging
    pose_estimator = PoseEstimator()

    # initial set up
    pose_init = np.array([0.0, 0.0, 0.0], dtype=float)
    time_init = 0.0
    pose_estimator.Reset(pose_init, time_init)

    # simulate
    time_end = 2.0
    t = np.arange(time_init, time_end, 0.01)
    N = len(t)

    pose = np.zeros(shape=(3,N),dtype=float)
    pose[:,0] = pose_init

    for k in range(0,N-1):
        veh_cmd = VehicleCommand(0.5, 2.0, t[k])
        pose_estimator.FeedCommandQueue2(veh_cmd)

    for k in range(0,N):
        pose[:,k], _ = pose_estimator.PredictState(t[k])

    # measurement update
    pose_meas = pose[:,-1] + np.array([0.05,0.05,0.1])
    pose_estimator.UpdateWithPoseMeasurement(pose_meas, 0.5*np.diag([1.0, 1.0, 1.0]),2.0)
    pose2, _ = pose_estimator.PredictState(2.0)

    # plot results
    fig = plt.figure()
    plt.plot(pose[0,:],pose[1,:])
    plt.grid(True)
    plt.axis('equal')
    plt.plot(pose_meas[0],pose_meas[1],'r*')
    plt.plot(pose2[0], pose2[1], 'g*')
    plt.show()

