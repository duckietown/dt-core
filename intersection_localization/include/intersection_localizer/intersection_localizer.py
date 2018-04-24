#!/usr/bin/env python
import rospy
import edge_model as em
import numpy as np
import duckietown_utils as dt
import cv2


class IntersectionLocalizer(object):
    '''class for localizing pose of Duckiebot at intersection'''

    def __init__(self, robot_name=''):
        self.robot_name = robot_name

        # camera parameters
        self.intrinsics = dt.load_camera_intrinsics(self.robot_name)
        homography = dt.load_homography(self.robot_name)
        self.H = np.linalg.inv(homography)

        # edge detection parameters
        self.canny_lower_threshold = 100  # self.SetupParameter("~canny_lower_threshold", 100)
        self.canny_upper_threshold = 200  # self.SetupParameter("~canny_upper_threshold", 200)
        self.canny_aperture_size = 3  # self.SetupParameter("~canny_aperture_size", 5)

        # visibility parameters
        self.border_cutoff = 10.0  # remove 10pixels around border
        self.x_dist_cutoff = 0.5 # remove things that are beyond 0.5m from Duckiebot

        pt_img_h = np.dot(self.H, np.array([self.x_dist_cutoff, 0.0, 1.0]))
        pt_img = pt_img_h[0:2]/pt_img_h[2]
        n_par_img = np.dot(self.H[0:2, 0:2],np.array([0.0, 1.0]))
        n_par_img = n_par_img/np.linalg.norm(n_par_img)
        n_perp_img = np.array([n_par_img[1],-n_par_img[0]])
        if n_perp_img[1] < 0.0:
            n_perp_img = -n_perp_img

        self.A_visible = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0], -n_perp_img])
        b_visible = np.array([- (0.0 + self.border_cutoff), 640.0 - self.border_cutoff, -(0.0 + self.border_cutoff),
                              480.0 - self.border_cutoff, -np.dot(pt_img, n_perp_img)])
        self.b_visible = b_visible[:, np.newaxis]
        self.mask_visible = np.zeros(shape=(480,640), dtype=np.uint8)

        pts_grid = np.meshgrid(np.arange(0,640),np.arange(0,480))
        pts = np.reshape(pts_grid,(2, 640*480))
        visible = np.all(np.dot(self.A_visible, pts) - self.b_visible < 0.0, 0)
        self.mask_visible[pts[1,visible],pts[0,visible]] = 255

        self.A_visible_img = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]])
        self.b_visible_img = np.array([0.0, 640.0, 0.0, 480.0])

        # weighting parameters
        self.weight_c = 5.0
        # TODO:
        # tune weight
        # multiple lines
        # distance from duckiebot

        # localization algorithm parameters
        self.line_search_length = 20
        self.max_num_iter = 2
        self.ctrl_pts_density = 80  # number of control points per edge length (in meters)
        self.min_num_ctrl_pts = 10
        self.max_num_ctrl_pts = 100

        # edge templates
        self.edge_models = {'THREE_WAY_INTERSECTION': em.EdgeModel('THREE_WAY_INTERSECTION', self.ctrl_pts_density),
                            'FOUR_WAY_INTERSECTION': em.EdgeModel('FOUR_WAY_INTERSECTION', self.ctrl_pts_density)}

        # auxiliary variables
        self.A_par_to_perp = np.array([[0.0, 1.0], [-1.0, 0.0]], dtype=float)

        px_offset = np.array([self.line_search_length, self.line_search_length])
        self.px_offset = px_offset[:, np.newaxis]

        self.G1 = np.zeros(shape=(3, 3), dtype=float)
        self.G1[0, 2] = 1

        self.G2 = np.zeros(shape=(3, 3), dtype=float)
        self.G2[1, 2] = 1

        self.G3 = np.zeros(shape=(3, 3), dtype=float)
        self.G3[0, 1] = 1
        self.G3[1, 0] = -1

    def ProcessRawImage(self, img_raw):
        # rectify image
        img_undistorted = dt.rectify(dt.rgb_from_ros(img_raw), self.intrinsics)

        # compute grayscale image
        img_gray = cv2.cvtColor(img_undistorted, cv2.COLOR_RGB2GRAY)

        # detect edges
        img_canny = cv2.Canny(img_gray, self.canny_lower_threshold, self.canny_upper_threshold,
                              apertureSize=self.canny_aperture_size)

        # apply mask
        img_mask = cv2.bitwise_and(img_canny,self.mask_visible)

        # pad image to avoid running into borders
        img_processed = cv2.copyMakeBorder(img_mask, self.line_search_length, self.line_search_length,
                                           self.line_search_length, self.line_search_length, cv2.BORDER_CONSTANT,
                                           value=0)

        return img_processed, img_gray

    def ComputeVisibleEdge(self, ptA, ptB):
        if np.all(np.dot(self.A_visible_img, ptA) - self.b_visible_img < 0.0):
            if np.all(np.dot(self.A_visible_img, ptB) - self.b_visible_img < 0.0):
                # both are visible
                return True, ptA, ptB

            else:
                # one is visible (there is always a solution)
                n = ptB - ptA
                An = np.dot(self.A_visible_img, n)
                res = (self.b_visible_img - np.dot(self.A_visible_img, ptA)) / An

                l_max = np.min(res[An > 0.0])

                return True, ptA, ptA + l_max * n

        else:
            if np.all(np.dot(self.A_visible_img, ptB) - self.b_visible_img < 0.0):
                # one is visible (there is always a solution)
                n = ptA - ptB
                An = np.dot(self.A_visible_img, n)
                res = (self.b_visible_img - np.dot(self.A_visible_img, ptB)) / An

                l_max = np.min(res[An > 0.0])

                return True, ptB + l_max * n, ptB

            else:
                # none is visible (make sure there is a solution)
                n = ptA - ptB
                An = np.dot(self.A_visible_img, n)
                res = (self.b_visible_img - np.dot(self.A_visible_img, ptB)) / An

                l_max = np.min([np.min(res[An > 0.0]), 1.0])
                l_min = np.max([np.max(res[An <= 0.0]), 0.0])

                if l_max < l_min:
                    return False, ptA, ptB
                else:
                    return True, ptB + l_max * n, ptB + l_min * n

    def DrawModel(self, img, x, y, theta, object_type):
        '''compute motion matrix'''
        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        t = np.dot(R, np.array([x, y], dtype=float))

        '''compute visible segments of edges'''
        edges_img = []
        for edge in self.edge_models[object_type].edges:
            ptA = np.dot(R, edge.ptA) - t
            ptB = np.dot(R, edge.ptB) - t

            # convert to image coordinates
            ptA_h_img = np.dot(self.H, np.hstack((ptA, 1)))
            ptB_h_img = np.dot(self.H, np.hstack((ptB, 1)))

            in_front_of_camera = False
            if ptA_h_img[2] < 0.0:
                in_front_of_camera = True

                ptA_img = ptA_h_img[0:2] / ptA_h_img[2]

                if ptB_h_img[2] < 0.0:
                    ptB_img = ptB_h_img[0:2] / ptB_h_img[2]

                else:
                    n_par = (ptB - ptA)
                    l = -ptA_h_img[2] / np.dot(self.H[2, :], np.hstack((n_par, 1)))
                    ptB_h_img = np.dot(self.H, np.hstack((ptA + 0.99 * l * n_par, 1)))
                    ptB_img = ptB_h_img[0:2] / ptB_h_img[2]

            elif ptB_h_img[2] < 0.0:
                in_front_of_camera = True

                ptB_img = ptB_h_img[0:2] / ptB_h_img[2]

                n_par = (ptA - ptB)
                l = -ptB_h_img[2] / np.dot(self.H[2, :], np.hstack((n_par, 1)))
                ptA_h_img = np.dot(self.H, np.hstack((ptB + 0.99 * l * n_par, 1)))
                ptA_img = ptA_h_img[0:2] / ptA_h_img[2]

            if in_front_of_camera:
                visible, ptA_img, ptB_img = self.ComputeVisibleEdge(ptA_img, ptB_img)
                if visible:
                    cv2.line(img, tuple(np.round(ptA_img).astype(np.int)), tuple(np.round(ptB_img).astype(np.int)), 255,
                             1)
                    cv2.circle(img, tuple(np.round(ptA_img).astype(np.int)), 3, 255, -1)
                    cv2.circle(img, tuple(np.round(ptB_img).astype(np.int)), 3, 255, -1)

    def ComputePose(self, img, x_pred, y_pred, theta_pred, object_type):
        # solve iterative least square
        for k in range(0, self.max_num_iter):
            '''compute motion matrix'''
            R = np.array([[np.cos(theta_pred), np.sin(theta_pred)], [-np.sin(theta_pred), np.cos(theta_pred)]])
            t = np.dot(R, np.array([x_pred, y_pred], dtype=float))

            T = np.zeros(shape=(3, 3), dtype=np.float)
            T[0:2, 0:2] = R
            T[0:2, 2] = -t
            T[2, 2] = 1

            '''project ctrl points'''
            ctrl_pts_h_img = np.dot(np.dot(self.H, T), self.edge_models[object_type].ctrl_pts_h)

            ''' compute visible ctrl points'''
            # need to be on the correct side of the image frame
            visible1 = ctrl_pts_h_img[2, :] < 0.0
            visible1_idx = np.arange(0, ctrl_pts_h_img.shape[1])[visible1]
            ctrl_pts_img = ctrl_pts_h_img[0:2, visible1] / ctrl_pts_h_img[2, visible1]

            # need to be in the range of the image
            visible2 = np.all(np.dot(self.A_visible, ctrl_pts_img) - self.b_visible < 0.0, 0)

            # reduce data to visible points
            ctrl_pts_h = self.edge_models[object_type].ctrl_pts_h[:, visible1_idx[visible2]]
            ctrl_pts_h_img = ctrl_pts_h_img[:, visible1_idx[visible2]]
            ctrl_pts_img = ctrl_pts_img[:, visible2]

            '''compute edge normal'''
            ctrl_pts_n_par_h_img = np.dot(np.dot(self.H[:, 0:2], R),
                                          self.edge_models[object_type].ctrl_pts_n_par[:, visible1_idx[visible2]])
            ctrl_pts_n_par_img = ctrl_pts_n_par_h_img[0:2, :] / ctrl_pts_n_par_h_img[2, :]
            ctrl_pts_n_perp = np.dot(self.A_par_to_perp, (ctrl_pts_n_par_img - ctrl_pts_img))
            ctrl_pts_n_perp = ctrl_pts_n_perp / np.linalg.norm(ctrl_pts_n_perp, 2, 0)

            '''compute distance from control points to closest edge along edge normal'''
            ctrl_pts_img_offset = ctrl_pts_img + self.px_offset

            # randomly iterate over all pts until maximum number of feasible points is reached
            idx = np.arange(0, ctrl_pts_img_offset.shape[1], dtype=int)
            np.random.shuffle(idx)

            num_pts = np.min([self.max_num_ctrl_pts, ctrl_pts_img_offset.shape[1]])
            dist = np.zeros(shape=(num_pts), dtype=float)
            num_feasible = 0
            idx_feasible = []
            for i in range(0, num_pts):
                for k in range(0, self.line_search_length):
                    if img[int(round(ctrl_pts_img_offset[1, i] + k * ctrl_pts_n_perp[1, i])),
                           int(round(ctrl_pts_img_offset[0, i] + k * ctrl_pts_n_perp[0, i]))]:
                        dist[num_feasible] = k
                        idx_feasible.append(i)
                        num_feasible += 1
                        break

                    elif img[int(round(ctrl_pts_img_offset[1, i] - k * ctrl_pts_n_perp[1, i])),
                             int(round(ctrl_pts_img_offset[0, i] - k * ctrl_pts_n_perp[0, i]))]:
                        dist[num_feasible] = -k
                        idx_feasible.append(i)
                        num_feasible += 1
                        break

            # remove invalid entries
            dist = dist[0:num_feasible]
            ctrl_pts_h_feasible = ctrl_pts_h[:, idx_feasible]
            ctrl_pts_img_feasible = ctrl_pts_img[:, idx_feasible]
            ctrl_pts_h_img_feasible = ctrl_pts_h_img[:, idx_feasible]
            ctrl_pts_n_perp_feasible = ctrl_pts_n_perp[:, idx_feasible]

            # check if there are enough ctrl points left
            if num_feasible < self.min_num_ctrl_pts:
                return False, 0.0, 0.0, 0.0

            '''compute gradients'''
            # compute L1
            ctrl_pts_h_img_1 = np.dot(np.dot(self.H, np.dot(self.G1, T)), ctrl_pts_h_feasible)
            ctrl_pts_img_1 = ctrl_pts_h_img_1[0:2, :] / ctrl_pts_h_img_1[2, :]
            L1 = (ctrl_pts_img_1 - ctrl_pts_img_feasible) * (ctrl_pts_h_img_1[2, :] / ctrl_pts_h_img_feasible[2, :])
            f1 = np.einsum('ij,ij->j', L1, ctrl_pts_n_perp_feasible)

            # compute L2
            ctrl_pts_h_img_2 = np.dot(np.dot(self.H, np.dot(self.G2, T)), ctrl_pts_h_feasible)
            ctrl_pts_img_2 = ctrl_pts_h_img_2[0:2, :] / ctrl_pts_h_img_2[2, :]
            L2 = (ctrl_pts_img_2 - ctrl_pts_img_feasible) * (ctrl_pts_h_img_2[2, :] / ctrl_pts_h_img_feasible[2, :])
            f2 = np.einsum('ij,ij->j', L2, ctrl_pts_n_perp_feasible)

            # compute L3
            ctrl_pts_h_img_3 = np.dot(np.dot(self.H, np.dot(self.G3, T)), ctrl_pts_h_feasible)
            ctrl_pts_img_3 = ctrl_pts_h_img_3[0:2, :] / ctrl_pts_h_img_3[2, :]
            L3 = (ctrl_pts_img_3 - ctrl_pts_img_feasible) * (ctrl_pts_h_img_3[2, :] / ctrl_pts_h_img_feasible[2, :])
            f3 = np.einsum('ij,ij->j', L3, ctrl_pts_n_perp_feasible)

            '''compute weights'''
            weight_sqrt = np.sqrt(1.0 / (np.abs(dist) + self.weight_c))
            W = np.diag(weight_sqrt)

            '''solve weighted least squares problem'''
            Aw = np.dot(W, np.array([f1, f2, f3]).T)
            bw = weight_sqrt * dist
            res = np.linalg.lstsq(Aw, bw)[0]

            '''update prediction'''
            t = np.dot(R.T, res[0:2])
            x_pred = x_pred - t[0]
            y_pred = y_pred - t[1]
            theta_pred = theta_pred + res[2]

            # TODO:
            # termination criteria
            # compute "covariance" of pose

        # debugging
        if 0:
            for i in range(0, ctrl_pts_img_offset.shape[1]):
                cv2.circle(img, tuple(np.round(ctrl_pts_img_offset[:, i]).astype(np.int)), 2, 180, -1)

            cv2.imshow('canny', img)

        return True, x_pred, y_pred, theta_pred

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("%s = %s " % (param_name, value))
        return value
