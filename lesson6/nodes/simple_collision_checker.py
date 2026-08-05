#!/usr/bin/env python3

import rospy
import shapely
import math
import numpy as np
import threading
from ros_numpy import msgify
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from autoware_mini.msg import Path, DetectedObjectArray, StopLineStatus, StopLineStatusArray
from sensor_msgs.msg import PointCloud2

DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('vx', np.float32),
    ('vy', np.float32),
    ('vz', np.float32),
    ('distance_to_stop', np.float32),
    ('deceleration_limit', np.float32),
    ('category', np.int32)
])


class SimpleCollisionChecker:

    def __init__(self):

        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal")
        self.braking_safety_distance_stopline = rospy.get_param("~braking_safety_distance_stopline")

        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')

        lanelet2_map = load(lanelet2_map_path, projector)
        self.stop_lines = self.get_traffic_light_stop_lines(lanelet2_map)

        self.detected_objects = None
        self.goal_point = None
        self.stopline_statuses = {}

        self.lock = threading.Lock()

        self.collision_points_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/detection/traffic_light_status', StopLineStatusArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def global_path_callback(self, msg):
        if len(msg.waypoints) > 0:
            self.goal_point = msg.waypoints[-1].position
        else:
            self.goal_point = None

    def traffic_light_status_callback(self, msg):
        with self.lock:
            self.stopline_statuses = {status.stop_line_id: status.status for status in msg.statuses}

    def path_callback(self, msg):
        with self.lock:
            detected_objects = self.detected_objects
            goal_point = self.goal_point
            stopline_statuses = self.stopline_statuses.copy()

        collision_points = np.array([], dtype=DTYPE)

        if len(msg.waypoints) == 0:
            collision_points_msg = PointCloud2()
            collision_points_msg.header = msg.header
            self.collision_points_pub.publish(collision_points_msg)
            return

        local_path_xyz = np.array([(wp.position.x, wp.position.y, wp.position.z) for wp in msg.waypoints])
        local_path_linestring = shapely.LineString(local_path_xyz)
        local_path_buffer = local_path_linestring.buffer(self.safety_box_width / 2, cap_style="flat")
        shapely.prepare(local_path_buffer)

        if detected_objects is not None and len(detected_objects) > 0:
            for obj in detected_objects:
                if len(obj.convex_hull) < 9:
                    continue

                object_points = np.array(obj.convex_hull).reshape(-1, 3)
                object_polygon = shapely.Polygon(object_points[:, :2])

                if local_path_buffer.intersects(object_polygon):
                    intersection = local_path_buffer.intersection(object_polygon)
                    intersection_points = shapely.get_coordinates(intersection)

                    object_speed = math.sqrt(obj.velocity.x ** 2 + obj.velocity.y ** 2 + obj.velocity.z ** 2)
                    category = 3 if object_speed < self.stopped_speed_limit else 4

                    for x, y in intersection_points:
                        collision_points = np.append(collision_points, np.array(
                            [(
                                x,
                                y,
                                obj.centroid.z,
                                obj.velocity.x,
                                obj.velocity.y,
                                obj.velocity.z,
                                self.braking_safety_distance_obstacle,
                                np.inf,
                                category
                            )], dtype=DTYPE))

        if goal_point is not None:
            goal_point_shapely = shapely.Point(goal_point.x, goal_point.y)

            if local_path_buffer.intersects(goal_point_shapely.buffer(0.1)):
                collision_points = np.append(collision_points, np.array(
                    [(
                        goal_point.x,
                        goal_point.y,
                        goal_point.z,
                        0.0,
                        0.0,
                        0.0,
                        self.braking_safety_distance_goal,
                        np.inf,
                        1
                    )], dtype=DTYPE))

        for stop_line_id, status in stopline_statuses.items():
            if status != StopLineStatus.STATUS_STOP:
                continue

            stop_line = self.stop_lines.get(stop_line_id)
            if stop_line is None:
                continue

            if local_path_linestring.intersects(stop_line):
                intersection = local_path_linestring.intersection(stop_line)
                intersection_points = shapely.get_coordinates(intersection)

                for x, y in intersection_points:
                    collision_points = np.append(collision_points, np.array(
                        [(
                            x,
                            y,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            self.braking_safety_distance_stopline,
                            np.inf,
                            2
                        )], dtype=DTYPE))

        if len(collision_points) > 0:
            collision_points_msg = msgify(PointCloud2, collision_points)
        else:
            collision_points_msg = PointCloud2()
        collision_points_msg.header = msg.header
        self.collision_points_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

    @staticmethod
    def get_traffic_light_stop_lines(lanelet2_map):
        stop_lines = {}
        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                stop_line = reg_el.parameters["ref_line"][0]
                stop_lines[stop_line.id] = shapely.LineString([(p.x, p.y) for p in stop_line])

        return stop_lines


if __name__ == '__main__':
    rospy.init_node('simple_collision_checker', log_level=rospy.INFO)
    node = SimpleCollisionChecker()
    node.run()
