import os
import math
import rospkg
import rospy
import geometry as geo

from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
import duckietown_world as dw
from duckietown_world.geo.measurements_utils import iterate_by_class

def get_tile_marker(x, y, angle, marker_type, marker_id, marker_scale):

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = 'tiles'

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD

    marker.pose.position.x = x 
    marker.pose.position.y = y
    marker.pose.position.z = -0.05

    marker.scale.x = marker_scale
    marker.scale.y = marker_scale
    marker.scale.z = 1

    marker.mesh_resource = "package://duckietown_visualization/meshes/tiles/" + \
        marker_type + ".dae"

    marker.mesh_use_embedded_materials = True

    q = quaternion_from_euler(0, 0, angle)

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    if marker_type == "curve_left" or marker_type == "curve_right":
       marker.scale.x = marker_scale * 1.025  # looks better
       marker.scale.y = marker_scale * 1.025

    return marker


def get_sign_marker(x, y, angle, marker_type, marker_id, marker_scale):

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = 'traffic-signs'

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = -0.04

    marker.scale.x = marker_scale
    marker.scale.y = marker_scale
    marker.scale.z = marker_scale

    marker.mesh_resource = "package://duckietown_visualization/meshes/traffic-signs/" + \
        marker_type + ".dae"

    marker.mesh_use_embedded_materials = True

    q = quaternion_from_euler(0, 0, angle)

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker


def get_apriltag_marker(x, y, angle, marker_type, marker_id, marker_scale):

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = 'apriltags'

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = -0.04

    marker.scale.x = 0.065
    marker.scale.y = 0.065
    marker.scale.z = 1

    marker.mesh_resource = "package://duckietown_visualization/meshes/apriltags/" + \
        marker_type + ".dae"

    marker.mesh_use_embedded_materials = True

    q = quaternion_from_euler(0, 0, angle)

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker


def get_marker_array_from_map(m):

    known_tile_types, known_sign_types = get_list_of_supported_types()

    marker_array = MarkerArray()
    marker_id = 0

    # Add all tiles
    records = list(iterate_by_class(m, dw.Tile))
    for record in records:
        tile = record.object
        matrix = record.transform_sequence.asmatrix2d().m
        translation, angle, scale = geo.translation_angle_scale_from_E2(matrix)
        
        if tile.kind not in known_tile_types:
            rospy.logerr("Unknown tile type: %s", tile.kind)
        
        marker = get_tile_marker(translation[0], translation[1], angle, 
            tile.kind, marker_id, scale)

        rospy.loginfo("Created marker for %s", tile.kind)
        marker_array.markers.append(marker)
        marker_id += 1

    marker_id = 0

    # Add all signs
    records = list(iterate_by_class(m, dw.Sign))
    for record in records:
        sign = record.object
        matrix = record.transform_sequence.asmatrix2d().m
        translation, angle, scale = geo.translation_angle_scale_from_E2(matrix)

        if sign.get_name_texture() not in known_sign_types:
            rospy.logerr("Unknown sign type: %s", sign.get_name_texture())

        marker = get_sign_marker(translation[0], translation[1], angle, 
            sign.get_name_texture(), marker_id, scale)

        rospy.loginfo("Created marker for %s", sign.get_name_texture())
        marker_array.markers.append(marker)
        marker_id += 1

    marker_id = 0

    # Add all apriltags
    records = list(iterate_by_class(m, dw.tags_db.FloorTag))
    for record in records:
        matrix = record.transform_sequence.asmatrix2d().m
        translation, angle, scale = geo.translation_angle_scale_from_E2(matrix)

        marker = get_apriltag_marker(translation[0], translation[1], angle, 
           "apriltag", marker_id, scale)

        rospy.loginfo("Created marker for Apriltag-%s", record.fqn[0])

        marker_array.markers.append(marker)
        marker_id += 1

    return marker_array


def get_list_of_supported_types():
    rospack = rospkg.RosPack()
    path = rospack.get_path('duckietown_visualization')

    tile_file_list = os.listdir(os.path.join(path, 'meshes', 'tiles'))
    sign_file_list = os.listdir(os.path.join(path, 'meshes', 'traffic-signs'))

    tile_types = set([f.split('.')[0] for f in tile_file_list])
    sign_types = set([f.split('.')[0] for f in sign_file_list])

    return tile_types, sign_types
