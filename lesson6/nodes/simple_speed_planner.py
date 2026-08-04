#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
import threading
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, LocalPath
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from simple_swerve_helper import SWERVE_SPEED_LIMIT, make_swerve_plan

toggle_swerve = False


class SimpleSpeedPlanner:

    def __init__(self):

        self.default_deceleration = rospy.get_param("default_deceleration")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")
        self.distance_to_car_front = rospy.get_param("distance_to_car_front")

        self.current_position = None
        self.current_speed = None

        self.lock = threading.Lock()

        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1, tcp_nodelay=True)
        self.visualized_path_pub = rospy.Publisher('visualized_path', LocalPath, queue_size=1, tcp_nodelay=True)
        self.swerve_point_markers_pub = rospy.Publisher('swerve_point_markers', MarkerArray, queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)
        ts.registerCallback(self.collision_points_and_path_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            with self.lock:
                collision_points = numpify(collision_points_msg) if len(collision_points_msg.data) > 0 else np.array([])
                current_position = self.current_position
                current_speed = self.current_speed

            if current_speed is None or current_position is None:
                rospy.logwarn_throttle(3, "%s - current speed or position not received!", rospy.get_name())
                return

            if len(local_path_msg.waypoints) == 0 or len(collision_points) == 0:
                self.publish_local_path(local_path_msg)
                self.publish_swerve_point_markers(local_path_msg.header)
                return

            local_path_xyz = np.array([(wp.position.x, wp.position.y, wp.position.z) for wp in local_path_msg.waypoints])
            local_path_linestring = shapely.LineString(local_path_xyz)

            collision_points_shapely = shapely.points(structured_to_unstructured(collision_points[['x', 'y', 'z']]))
            collision_point_distances = np.array([local_path_linestring.project(cp) for cp in collision_points_shapely])
            ego_distance_from_local_path_start = local_path_linestring.project(current_position)
            collision_point_distances_from_ego = collision_point_distances - ego_distance_from_local_path_start

            calculated_target_velocities = np.sqrt(
                np.maximum(0, 2 * self.default_deceleration * collision_point_distances_from_ego)
            )
            target_idx = np.argmin(calculated_target_velocities)

            target_velocity = calculated_target_velocities[target_idx]
            target_object_distance = collision_point_distances_from_ego[target_idx]
            target_object_speed = 0
            stopping_point_distance = collision_point_distances[target_idx]
            collision_point_category = collision_points[target_idx]["category"]

            collision_point_braking_distances = collision_points["distance_to_stop"]
            target_distances = (
                collision_point_distances_from_ego
                - self.distance_to_car_front
                - collision_point_braking_distances
            )
            calculated_target_velocities = np.sqrt(
                np.maximum(0, 2 * self.default_deceleration * target_distances)
            )
            target_idx = np.argmin(calculated_target_velocities)

            target_velocity = calculated_target_velocities[target_idx]
            target_object_distance = collision_point_distances_from_ego[target_idx] - self.distance_to_car_front
            stopping_point_distance = collision_point_distances[target_idx] - collision_points["distance_to_stop"][target_idx]
            collision_point_category = collision_points[target_idx]["category"]

            collision_point_path_headings = [
                self.get_heading_at_distance(local_path_linestring, d)
                for d in collision_point_distances
            ]
            collision_point_speeds = np.array([
                self.project_vector_to_heading(heading, Vector3(vx, vy, vz))
                for heading, (vx, vy, vz) in zip(
                    collision_point_path_headings,
                    collision_points[["vx", "vy", "vz"]]
                )
            ])

            target_distances = target_distances - self.braking_reaction_time * np.abs(collision_point_speeds)

            approaching_speeds = np.minimum(collision_point_speeds, 0)
            calculated_target_velocities = np.maximum(
                0,
                approaching_speeds + np.sqrt(np.maximum(
                    0,
                    collision_point_speeds ** 2 + 2 * self.default_deceleration * target_distances
                ))
            )

            target_idx = np.argmin(calculated_target_velocities)

            target_velocity = calculated_target_velocities[target_idx]
            target_object_distance = collision_point_distances_from_ego[target_idx] - self.distance_to_car_front
            target_object_speed = collision_point_speeds[target_idx]
            stopping_point_distance = collision_point_distances[target_idx] - collision_points["distance_to_stop"][target_idx]
            collision_point_category = collision_points[target_idx]["category"]

            if toggle_swerve and collision_point_category in (3, 4):
                swerve_plan = make_swerve_plan(
                    local_path_msg,
                    local_path_linestring,
                    collision_points,
                    collision_point_distances,
                    target_idx,
                    ego_distance_from_local_path_start,
                    self.distance_to_car_front
                )

                if swerve_plan is not None:
                    for wp in local_path_msg.waypoints:
                        wp.speed = min(wp.speed, SWERVE_SPEED_LIMIT)

                    path = Path()
                    path.header = local_path_msg.header
                    path.waypoints = local_path_msg.waypoints

                    self.publish_local_path(path,
                                            target_object_distance=swerve_plan.return_distance - ego_distance_from_local_path_start - self.distance_to_car_front,
                                            target_object_speed=0.0,
                                            stopping_point_distance=swerve_plan.return_distance,
                                            collision_point_category=collision_point_category,
                                            is_blocked=True)
                    self.publish_swerve_point_markers(local_path_msg.header,
                                                      local_path_linestring,
                                                      swerve_plan.start_distance,
                                                      swerve_plan.return_distance)
                    return

            zero_speeds_onwards = False
            target_distance_object = target_distances[target_idx] + ego_distance_from_local_path_start
            approaching_speed = min(target_object_speed, 0.0)

            for i, wp in enumerate(local_path_msg.waypoints):
                if zero_speeds_onwards:
                    wp.speed = 0.0
                    continue

                if i > 0:
                    previous_wp = local_path_msg.waypoints[i - 1]
                    target_distance_object -= math.sqrt(
                        (wp.position.x - previous_wp.position.x) ** 2 +
                        (wp.position.y - previous_wp.position.y) ** 2
                    )

                target_speed_object = max(
                    0.0,
                    approaching_speed + math.sqrt(max(
                        0.0,
                        target_object_speed ** 2 + 2 * self.default_deceleration * target_distance_object
                    ))
                )

                wp.speed = min(target_speed_object, wp.speed)

                if math.isclose(wp.speed, 0.0):
                    zero_speeds_onwards = True

            path = Path()
            path.header = local_path_msg.header
            path.waypoints = local_path_msg.waypoints

            self.publish_local_path(path,
                                    target_object_distance=target_object_distance,
                                    target_object_speed=target_object_speed,
                                    stopping_point_distance=stopping_point_distance,
                                    collision_point_category=collision_point_category,
                                    is_blocked=True)
            self.publish_swerve_point_markers(local_path_msg.header)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def publish_local_path(self, path, target_object_distance=0.0, target_object_speed=0.0,
                           stopping_point_distance=0.0, collision_point_category=0, is_blocked=False):
        self.local_path_pub.publish(path)
        self.visualized_path_pub.publish(LocalPath(header=path.header, waypoints=path.waypoints,
                                                   target_object_distance=float(target_object_distance),
                                                   target_object_speed=float(target_object_speed),
                                                   stopping_point_distance=float(stopping_point_distance),
                                                   collision_point_category=int(collision_point_category),
                                                   is_blocked=bool(is_blocked)))

    def publish_swerve_point_markers(self, header, linestring=None, start_distance=0.0, end_distance=0.0):
        marker_array = MarkerArray()

        if linestring is None:
            marker = Marker(header=header)
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            self.swerve_point_markers_pub.publish(marker_array)
            return

        marker_array.markers.append(self.make_swerve_line_marker(header, linestring, start_distance, 0,
                                                                 ColorRGBA(1.0, 0.0, 0.0, 0.7)))
        marker_array.markers.append(self.make_swerve_line_marker(header, linestring, end_distance, 1,
                                                                 ColorRGBA(0.0, 1.0, 0.0, 0.7)))
        self.swerve_point_markers_pub.publish(marker_array)

    def make_swerve_line_marker(self, header, linestring, distance, marker_id, color):
        point = linestring.interpolate(distance)
        heading = self.get_heading_at_distance(linestring, distance)
        marker = Marker(header=header)
        marker.ns = "Swerve points"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z + 1.0
        marker.pose.orientation.z = math.sin(heading / 2.0)
        marker.pose.orientation.w = math.cos(heading / 2.0)
        marker.scale.x = 0.3
        marker.scale.y = 5.0
        marker.scale.z = 2.5
        marker.color = color
        marker.lifetime = rospy.Duration(0.3)
        return marker

    @staticmethod
    def get_heading_at_distance(linestring, distance):
        point_after = linestring.interpolate(distance + 0.1)
        point_before = linestring.interpolate(max(0, distance - 0.1))
        return math.atan2(point_after.y - point_before.y, point_after.x - point_before.x)

    @staticmethod
    def project_vector_to_heading(heading_angle, vector):
        return vector.x * math.cos(heading_angle) + vector.y * math.sin(heading_angle)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('simple_speed_planner', log_level=rospy.INFO)
    node = SimpleSpeedPlanner()
    node.run()
