class Point:
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    @static_method
    def from_message(msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

class GroundProjectionGeometry:

    def __init__(self, im_width, im_height, homography):
        self.im_width = im_width
        self.im_height = im_height
        self.H = homography
        self.Hinv = np.linalg.inv(self.H)

    def vector2pixel(self, vec):
        """ Converts a [0,1] X [0,1] representation to [0, W] X [0, H]. """
        x = self.camera_info.width * vec.x
        y = self.camera_info.height * vec.y
        return Point(x,y)

    def pixel2vector(self, pixel):
        """ Converts a [0,W] X [0,H] representation to [0, 1] X [0, 1]. """
        x = pixel.x / self.camera_info.width
        y = pixel.y / self.camera_info.height
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