import numpy as np
import cv2

class Point:

    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_message(msg):
        x = msg.x
        y = msg.y
        try:
            z = msg.z
        except AttributeError:
            z = 0
        return Point(x, y, z)


class GroundProjectionGeometry:

    def __init__(self, im_width, im_height, homography):
        self.im_width = im_width
        self.im_height = im_height
        self.H = homography
        self.Hinv = np.linalg.inv(self.H)

    def vector2pixel(self, vec):
        """ Converts a [0,1] X [0,1] representation to [0, W] X [0, H]. """
        x = self.im_width * vec.x
        y = self.im_height * vec.y
        return Point(x, y)

    def pixel2vector(self, pixel):
        """ Converts a [0,W] X [0,H] representation to [0, 1] X [0, 1]. """
        x = pixel.x / self.im_width
        y = pixel.y / self.im_height
        return Point(x, y)

    def pixel2ground(self, pixel):
        uv_raw = np.array([pixel.x, pixel.y, 1.0])
        ground_point = np.dot(self.H, uv_raw)
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x / z
        point.y = y / z
        point.z = 0.0
        return point

    def ground2pixel(self, point):
        if point.z != 0:
            msg = 'This method assumes that the point is a ground point (z=0). '
            msg += 'However, the point is (%s,%s,%s)' % (point.x, point.y, point.z)
            raise ValueError(msg)

        ground_point = np.array([point.x, point.y, 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]

        pixel = Point()
        pixel.x = image_point[0]
        pixel.y = image_point[1]

        return pixel

    @staticmethod
    def estimate_homography(cv_image_rectified, board_w, board_h, square_size, x_offset, y_offset):
        """Image should be rectified."""

        cv_image_rectified = cv2.cvtColor(cv_image_rectified, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(cv_image_rectified, (board_w, board_h),
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == False:
            raise RuntimeError("No corners found in image, or the corners couldn't be rearranged. Make sure that the "
                               "camera is positioned correctly.")

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        corners_subpix = cv2.cornerSubPix(cv_image_rectified, corners, (11, 11), (-1, -1), criteria)

        src_pts = []
        for r in range(board_h):
            for c in range(board_w):
                src_pts.append(
                    np.array([r * square_size, c * square_size], dtype='float32') + np.array([x_offset -y_offset]))

        # OpenCV labels corners left-to-right, top-to-bottom
        # We're having a problem with our pattern since it's not rotation-invariant

        # only reverse order if first point is at bottom right corner
        if (corners[0])[0][0] < (corners[self.board_['width'] * self.board_['height'] - 1])[0][0] and \
           (corners[0])[0][0] < (corners[self.board_['width'] * self.board_['height'] - 1])[0][1]:
            src_pts.reverse()

        # Compute homography from image to ground
        H, _ = cv2.findHomography(corners_subpix.reshape(len(corners_subpix), 2), np.array(src_pts), cv2.RANSAC)

        if (self.H[1][2] > 0):
            status = "Homography could be corrupt!"
        else:
            status = None

        return GroundProjectionGeometry(im_height=cv_image_rectified.shape[1],
                                        im_width=cv_image_rectified.shape[0],
                                        homography=H), \
               status
