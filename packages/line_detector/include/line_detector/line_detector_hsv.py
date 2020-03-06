import cv2
import numpy as np
from collections import namedtuple

Detections = namedtuple('Detections',
                        ['lines', 'normals', 'area', 'centers'])


class LineDetectorHSV():

    def __init__(self, canny_thresholds, canny_aperture_size, dilation_kernel_size,
                 hough_threshold, hough_min_line_length, hough_max_line_gap):

        self.canny_thresholds = canny_thresholds
        self.canny_aperture_size = canny_aperture_size
        self.dilation_kernel_size = dilation_kernel_size
        self.hough_threshold = hough_threshold
        self.hough_min_line_length = hough_min_line_length
        self.hough_max_line_gap = hough_max_line_gap

        # initialize the variables that will hold the processed images
        self.bgr = np.empty(0)
        self.hsv = np.empty(0)
        self.canny_edges = np.empty(0)

    def setImage(self, image):
        """
        Sets the ``bgr`` attribute to the provided image. Also stores
        an `HSV <https://en.wikipedia.org/wiki/HSL_and_HSV>`_ representation of the image and the
        extracted `Canny edges <https://en.wikipedia.org/wiki/Canny_edge_detector>`_. This is separated from
        :py:meth:`detectLines` so that the HSV representation and the edge extraction can be reused for multiple
        colors.

        Args:
            image (:obj:`numpy array`): input image

        """

        self.bgr = np.copy(image)
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.canny_edges = self.findEdges(self.bgr)

    def getImage(self):
        """
        Provides the image currently stored in the ``bgr`` attribute.

        Returns:
            :obj:`numpy array`: the stored image
        """
        return self.bgr

    def findEdges(self):
        """
        Applies `Canny edge detection <https://en.wikipedia.org/wiki/Canny_edge_detector>`_ to a ``BGR`` image.

        Args:
            bgr (:obj:`numpy array`): the input image

        Returns:
            :obj:`numpy array`: a binary image with the edges
        """
        edges = cv2.Canny(self.bgr, self.canny_thresholds[0], self.canny_thresholds[1],
                          apertureSize=self.canny_aperture_size)
        return edges

    def houghLine(self, edges):
        """
        Finds line segments in a binary image using the probabilistic Hough transform. Based on the OpenCV function
        `HoughLinesP <https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp
        #houghlinesp>`_.

        Args:
            edges (:obj:`numpy array`): binary image with edges

        Returns: :obj:`numpy array`: An ``Nx4`` array where each row represents a line. If no lines were detected,
        returns an empty list.

        """
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=self.hough_threshold,
                                minLineLength=self.hough_min_line_length, maxLineGap=self.hough_max_line_gap)
        if lines is not None:
            lines = lines.squeeze()  # it has an extra dimension
        else:
            lines = []

        return lines

    def colorFilter(self, color_range):
        """
        Obtains the regions of the image that fall in the provided color range and the subset of the detected Canny
        edges which are in these regions. Applies a `dilation <https://homepages.inf.ed.ac.uk/rbf/HIPR2/dilate.htm>`_
        operation to smooth and grow the regions map.

        Args:
            :py:class:`ColorRange`: A :py:class:`ColorRange` object specifying the desired colors.

        Returns:
            :obj:`numpy array`: binary image with the regions of the image that fall in the color range

            :obj:`numpy array`: binary image with the edges in the image that fall in the color range
        """
        # threshold colors in HSV space
        bw = color_range.inRange(self.hsv)

        # binary dilation: fills in gaps and makes the detected regions grow
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)

        # extract only the edges which come from the region with the selected color
        edge_color = cv2.bitwise_and(bw, self.canny_edges)

        return bw, edge_color

    def checkBounds(self, val, bound):
        """

        :param val:
        :param bound:
        :return:
        """
        val[val < 0] = 0
        val[val >= bound] = bound - 1
        return val

    def findNormal(self, lines):
        """

        :param lines:
        :return:
        """
        normals = []
        centers = []
        if len(lines) > 0:
            length = np.sum((lines[:, 0:2] - lines[:, 2:4]) ** 2, axis=1, keepdims=True) ** 0.5
            dx = 1. * (lines[:, 3:4] - lines[:, 1:2]) / length
            dy = 1. * (lines[:, 0:1] - lines[:, 2:3]) / length

            centers = np.hstack([(lines[:, 0:1] + lines[:, 2:3]) / 2, (lines[:, 1:2] + lines[:, 3:4]) / 2])
            x3 = (centers[:, 0:1] - 3. * dx).astype('int')
            y3 = (centers[:, 1:2] - 3. * dy).astype('int')
            x4 = (centers[:, 0:1] + 3. * dx).astype('int')
            y4 = (centers[:, 1:2] + 3. * dy).astype('int')
            x3 = self.checkBounds(x3, self.bgr.shape[1])
            y3 = self.checkBounds(y3, self.bgr.shape[0])
            x4 = self.checkBounds(x4, self.bgr.shape[1])
            y4 = self.checkBounds(y4, self.bgr.shape[0])
            flag_signs = (np.logical_and(bw[y3, x3] > 0, bw[y4, x4] == 0)).astype('int') * 2 - 1
            normals = np.hstack([dx, dy]) * flag_signs

        return centers, normals

    def detectLines(self, color_range):
        """

        :param color:
        :return:
        """
        bw, edge_color = self.colorFilter(color_range)
        lines = self.houghLine(edge_color)
        centers, normals = self.findNormal(bw, lines)
        return Detections(lines=lines, normals=normals, area=bw, centers=centers)
