#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import duckietown_utils as dt
import cv2


class Path(object):
    '''class describing path for Duckiebot'''

    def __init__(self, pos_init, dir_init, alpha_init, pos_final, dir_final, alpha_final):
        delta_pos = pos_final - pos_init - alpha_init * dir_init
        delta_vel = alpha_final * dir_final - alpha_init * dir_init

        # path coefficients
        self.coeffs_pos = np.zeros(shape=(2, 4), dtype=float)
        for i in range(0, 2):
            c_temp = np.dot(np.array([[-2.0, 1.0], [3.0, -1.0]]), np.array([delta_pos[i], delta_vel[i]]))
            self.coeffs_pos[i, 0] = c_temp[0]
            self.coeffs_pos[i, 1] = c_temp[1]

            self.coeffs_pos[i, 2] = alpha_init * dir_init[i]
            self.coeffs_pos[i, 3] = pos_init[i]

        self.coeffs_vel = np.array([np.polyder(self.coeffs_pos[0,:]),
                                    np.polyder(self.coeffs_pos[1,:])])
        self.coeffs_acc = np.array([np.polyder(self.coeffs_vel[0,:]),
                                    np.polyder(self.coeffs_vel[1,:])])
        self.coeffs_jerk = np.array([np.polyder(self.coeffs_acc[0,:]),
                                    np.polyder(self.coeffs_acc[1,:])])

        # curvature coefficients
        self.coeffs_num = np.polysub(np.polymul(self.coeffs_vel[0, :], self.coeffs_acc[1, :]),
                                     np.polymul(self.coeffs_vel[1, :], self.coeffs_acc[0, :]))
        self.coeffs_denom = np.polyadd(np.polymul(self.coeffs_vel[0, :], self.coeffs_vel[0, :]),
                                       np.polymul(self.coeffs_vel[1, :], self.coeffs_vel[1, :]))

        # roots for minimizing curvature
        self.roots_init = False


    def Evaluate(self, s):
        pos = np.array([np.polyval(self.coeffs_pos[0, :], s), np.polyval(self.coeffs_pos[1, :], s)])
        vel = np.array([np.polyval(self.coeffs_vel[0, :], s), np.polyval(self.coeffs_vel[1, :], s)])

        return pos, vel


    def EvaluateNumerator(self, s):
        return np.abs(np.polyval(self.coeffs_num, s))


    def EvaluateDenominator(self, s):
        return np.power(np.polyval(self.coeffs_denom, s), 1.5)


    def EvaluateCurvature(self, s):
        return np.abs(np.polyval(self.coeffs_num, s)) / np.power(np.polyval(self.coeffs_denom, s), 1.5)


    def EvaluateCurvatureWithSign(self, s):
        return np.polyval(self.coeffs_num, s) / np.power(np.polyval(self.coeffs_denom, s), 1.5)


    def ComputeRoots(self):
        self.roots_init = True

        coeffs_num_der = np.polyder(self.coeffs_num)
        self.roots_num = np.roots(coeffs_num_der)
        self.roots_num = self.roots_num[np.isreal(self.roots_num)]
        if self.roots_num.shape[0]:
            self.roots_num = self.roots_num[self.roots_num < 0.0]
        if self.roots_num.shape[0]:
            self.roots_num = self.roots_num[self.roots_num > 1.0]

        self.roots_num = np.sort(self.roots_num)

        coeffs_denom_der = np.polyder(self.coeffs_denom)
        self.roots_denom = np.roots(coeffs_denom_der)
        self.roots_denom = self.roots_denom[np.isreal(self.roots_denom)]
        if self.roots_denom.shape[0]:
            self.roots_denom = self.roots_denom[self.roots_denom < 0.0]
        if self.roots_denom.shape[0]:
            self.roots_denom = self.roots_denom[self.roots_denom > 1.0]

        self.roots_denom = np.sort(self.roots_denom)


    def IsFeasible(self):
        # check if path contains circle
        a = self.coeffs_pos[0,0]
        b = self.coeffs_pos[0,1]
        c = self.coeffs_pos[0,2]

        u = self.coeffs_pos[1,0]
        v = self.coeffs_pos[1,1]
        w = self.coeffs_pos[1,2]

        bumav = b*u - a*v
        if np.abs(bumav) > 1e-6:
            det = -bumav*bumav * (3.0*c*c*u*u + w*(4.0*b*b*u - 4.0*a*b*v + 3.0*a*a*w) +
                                     c*(-4.0*b*u*v + 4.0*a*v*v - 6.0*a*u*w))
            if det > 0:
                s = -(bumav*(c*u - a*w) + np.sqrt(det))/(2*bumav*bumav)
                t = (-bumav*(c*u - a*w) + np.sqrt(det))/(2*bumav*bumav)

                if 0.0 < s and s < 1.0 and 0.0 < t and t < 1.0:
                    return False

        # TODO: add other feasibility checks here, e.g. position constraints

        return True


class PathPlanner(object):
    '''class for planning paths for a Duckiebot to cross an intersection'''

    def __init__(self, robot_name=''):
        self.robot_name = robot_name

        # max curvature parameters
        self.maximum_feasible_curvature = 1e6  # TODO: adjust this number!
        self.num_intervals = 32
        self.alpha_min = 0.1
        self.alpha_max = 2.0
        self.num_alphas = 20
        self.alphas = np.linspace(self.alpha_min, self.alpha_max, self.num_alphas)

        # lane error parameters
        self.max_iterations = 20
        self.init_grad_step_size = 1.0
        self.step_size_increase = 1.2
        self.step_size_decrease = 0.5

        # drawing parameters
        homography = dt.load_homography(self.robot_name)
        self.H = np.linalg.inv(homography)

    def PlanPath(self, pose_init, pose_final):
        pos_init = np.array([pose_init[0], pose_init[1]], dtype=float)
        dir_init = np.array([np.cos(pose_init[2]), np.sin(pose_init[2])], dtype=float)

        pos_final = np.array([pose_final[0], pose_final[1]], dtype=float)
        dir_final = np.array([np.cos(pose_final[2]), np.sin(pose_final[2])], dtype=float)

        foundPath = False
        best_curvature_max = self.maximum_feasible_curvature
        for alpha_init in self.alphas:
            for alpha_final in self.alphas:

                # generate path
                path = Path(pos_init, dir_init, alpha_init, pos_final, dir_final, alpha_final)
                path.ComputeRoots()

                # evaluate numerator, denominator of curvature
                s = np.linspace(0.0, 1.0, self.num_intervals + 1)
                val_num = path.EvaluateNumerator(s)
                val_num_roots = path.EvaluateNumerator(path.roots_num)
                val_denom = path.EvaluateDenominator(s)
                val_denom_roots = path.EvaluateDenominator(path.roots_denom)

                # bound numerator, denominator
                bound_num = np.zeros(shape=(1, self.num_intervals), dtype=float)
                roots_num = np.append(path.roots_num, 10.0)
                idx_roots_num = 0

                bound_denom = np.zeros(shape=(1, self.num_intervals), dtype=float)
                roots_denom = np.append(path.roots_denom, 10.0)
                idx_roots_denom = 0
                for i in range(0, self.num_intervals):
                    if roots_num[idx_roots_num] < s[i + 1]:
                        bound_num[0, i] = max([val_num[i], val_num[i + 1], val_num_roots[idx_roots_num]])
                        idx_roots_num += 1
                    else:
                        bound_num[0, i] = np.max(val_num[i:i + 2])

                    if roots_denom[idx_roots_denom] < s[i + 1]:
                        bound_denom[0, i] = min([val_denom[i], val_denom[i + 1], val_denom_roots[idx_roots_denom]])
                        idx_roots_denom += 1
                    else:
                        bound_denom[0, i] = np.min(val_denom[i:i + 2])

                # compute (conservative) estimate of maximum curvature
                curvature_max = np.max(bound_num / bound_denom)

                if curvature_max < best_curvature_max and path.IsFeasible():
                    foundPath = True
                    best_curvature_max = curvature_max
                    self.path = path
                    self.curvature_max = curvature_max
                    self.alpha_init = alpha_init
                    self.alpha_final = alpha_final

        return foundPath

    def EvaluatePath(self, s):
        return self.path.Evaluate(s)

    def ComputeLaneError(self, pose, s):
        pos = np.array([pose[0], pose[1]], dtype=float)
        dir = np.array([np.cos(pose[2]), np.sin(pose[2])], dtype=float)

        pos_path, vel_path = self.path.Evaluate(s)
        dist = np.linalg.norm(pos - pos_path)

        dist_step_size = self.init_grad_step_size
        for k in range(0, self.max_iterations):
            # compute gradient
            grad_dist = -2.0 * ((pos[0] - pos_path[0]) * vel_path[0] +
                                (pos[1] - pos_path[1]) * vel_path[1])
            s_new = self.constrain(s - dist_step_size * grad_dist, 0.0, 1.0)

            pos_path_new, vel_path_new = self.path.Evaluate(s_new)
            dist_new = np.linalg.norm(pos - pos_path_new)

            if dist_new < dist:
                if dist < 1e-3 or (dist - dist_new) / dist < 1e-6:
                    # compute direction error
                    dir_path = vel_path_new / np.linalg.norm(vel_path_new)
                    theta = np.arctan2(dir_path[0] * dir[1] - dir_path[1] * dir[0],
                                       dir_path[0] * dir[0] + dir_path[1] * dir[1])

                    # compute curvature
                    curvature = self.path.EvaluateCurvatureWithSign(s_new)

                    return dist_new, theta, curvature, s_new

                pos_path[:] = pos_path_new
                vel_path[:] = vel_path_new
                dist = dist_new
                s = s_new
                dist_step_size *= self.step_size_increase

            elif dist == dist_new:
                if s == 0.0 or s == 1.0:
                    # compute direction error
                    dir_path = vel_path_new / np.linalg.norm(vel_path_new)
                    theta = np.arctan2(dir_path[0] * dir[1] - dir_path[1] * dir[0],
                                       dir_path[0] * dir[0] + dir_path[1] * dir[1])

                    # compute curvature
                    curvature = self.path.EvaluateCurvatureWithSign(s_new)

                    return dist_new, theta, curvature, s_new

                else:
                    pos_path[:] = pos_path_new
                    vel_path[:] = vel_path_new
                    dist = dist_new
                    s = s_new

            else:
                dist_step_size *= self.step_size_decrease

            if k == self.max_iterations - 1:
                # did not converge

                # compute direction error
                dir_path = vel_path_new / np.linalg.norm(vel_path_new)
                theta = np.arctan2(dir_path[0] * dir[1] - dir_path[1] * dir[0],
                                   dir_path[0] * dir[0] + dir_path[1] * dir[1])

                # compute curvatureEvaluateCurvature
                curvature = self.path.EvaluateCurvatureWithSign(s_new)

                return dist_new, theta, curvature, s_new

    def DrawPath(self, img, pose_current):
        num_segments = 40
        s = np.linspace(0.0, 1.0, num_segments + 1)

        # compute points along path
        pts, _ = self.path.Evaluate(s)

        # compute motion matrix
        R = np.array(
            [[np.cos(pose_current[2]), np.sin(pose_current[2])], [-np.sin(pose_current[2]), np.cos(pose_current[2])]])
        t = np.dot(R, np.array([pose_current[0], pose_current[1]], dtype=float))

        # project points into image frame
        pts_h = np.zeros(shape=(3, num_segments + 1), dtype=float)
        pts_h[0:2, :] = np.dot(R, pts) - t[:, np.newaxis]
        pts_h[2, :] = 1

        # convert to image points
        pts_img_h = np.dot(self.H, pts_h)
        pts_img = pts_img_h[0:2, :] / pts_img_h[2, :]

        # draw segments
        for i in range(0, num_segments):
            if pts_img_h[2, i] < 0.0 and pts_img_h[2, i + 1] < 0.0:
                cv2.line(img, tuple(np.round(pts_img[:, i]).astype(np.int)),
                         tuple(np.round(pts_img[:, i + 1]).astype(np.int)), 255, 3)

    def constrain(self, val, min_val, max_val):

        if val < min_val: return min_val
        if val > max_val: return max_val
        return val


if __name__ == '__main__':
    path_planner = PathPlanner('daisy')

    # set up
    pose_init = [0.4, -0.105, 1.2 * np.pi / 2.0]
    pose_final = [0.0508, 0.4, np.pi]

    # plan path
    ret = path_planner.PlanPath(pose_init, pose_final)

    if ret:
        print('Found path, plotting results.')
        s = np.linspace(0.0, 1.0, 40)
        pts, _ = path_planner.EvaluatePath(s)

        fig = plt.figure()
        plt.plot(pts[0, :], pts[1, :])
        plt.grid(True)
        plt.axis('equal')
        plt.axis([-0.4, 0.8, -0.4, 0.8])

    # compute lane error
    s_guess = 0.5
    pose_robot = [0.3, 0.4, 1.0 * np.pi / 2.0]
    d, theta, curvature, s_closest = path_planner.ComputeLaneError(pose_robot, s_guess)
    print('d:', d)
    print('theta:', theta / np.pi * 180.0)
    print('curvature:', curvature)

    # plot lane error results
    pos_closest, vel_closest = path_planner.EvaluatePath(s_closest)
    dir_robot = np.array([np.cos(pose_robot[2]), np.sin(pose_robot[2])], dtype=float)
    dir_closest = vel_closest / np.linalg.norm(vel_closest)

    plt.plot([pose_robot[0], pos_closest[0]], [pose_robot[1], pos_closest[1]], 'r')
    plt.plot([pose_robot[0], pose_robot[0] + 0.1 * dir_robot[0]], [pose_robot[1], pose_robot[1] + 0.1 * dir_robot[1]],
             'g')
    plt.plot(pose_robot[0], pose_robot[1], 'g*')

    plt.plot([pos_closest[0], pos_closest[0] + 0.1 * dir_closest[0]],
             [pos_closest[1], pos_closest[1] + 0.1 * dir_closest[1]], 'g')
    plt.plot(pos_closest[0], pos_closest[1], 'g*')

    plt.show()
