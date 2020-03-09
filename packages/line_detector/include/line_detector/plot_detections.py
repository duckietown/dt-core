import cv2
import numpy as np

def plotDetections(image, detections):
    """

    Args:
        image (:obj:`numpy array`):
        detections (`dict`): A dictionary that has keys :py:class:`ColorRange` and values :py:class:`Detection`

    Returns:

    """

    im = np.copy(image)

    for color_range, det in detections.iteritems():

        # convert HSV color to BGR
        c = color_range.representative
        c = np.uint8([[[c[0], c[1], c[2]]]])
        color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int)

        # plot all detected line segments and their normals
        for i in range(len(det.normals)):
            center = det.centers[i]
            normal = det.normals[i]
            im = cv2.line(im, tuple(center), tuple((center+10*normal).astype(int)), color=(0,0,0), thickness=2)
            # im = cv2.circle(im, (center[0], center[1]), radius=3, color=color, thickness=-1)
        for line in det.lines:
            im = cv2.line(im, (line[0], line[1]), (line[2], line[3]), color=(0,0,0), thickness=5)
            im = cv2.line(im, (line[0], line[1]), (line[2], line[3]), color=color, thickness=2)
    return im

