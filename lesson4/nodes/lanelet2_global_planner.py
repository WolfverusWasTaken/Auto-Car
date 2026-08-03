#!/usr/bin/env python3

import numpy as np
import rospy
from threading import Lock

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
import lanelet2.traffic_rules
import lanelet2.routing

class GlobalPlanner:
    def __init__(self):

        # Parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")
        self.distance_to_goal_limit = rospy.get_param("lanelet2_global_planner/distance_to_goal_limit")
        self.default_deceleration = rospy.get_param("default_deceleration")

        # Load Lanelet2 map
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_path, projector)

        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                    lanelet2.traffic_rules.Participants.VehicleTaxi)
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Internal variables
        self.lock = Lock()
        self.current_location = None
        self.goal_point = None

        # Publishers
        self.global_path_pub = rospy.Publisher('global_path', Path, latch=True, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def goal_callback(self, msg):
        with self.lock:
            self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        rospy.loginfo("%s - goal position (%f, %f, %f) in %s frame", rospy.get_name(),
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.header.frame_id)

        if self.current_location is None:
            return

        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]

        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, False)
        if route is None:
            rospy.logwarn("%s - No route found to goal position", rospy.get_name())
            return

        path = route.shortestPath()
        if path is None:
            rospy.logwarn("%s - No shortest path found", rospy.get_name())
            return

        laneletseq = path.getRemainingLane(start_lanelet)

        waypoints = self.convert_laneletseq_to_waypoints_list(laneletseq)
        self.publish_lane_from_waypoints_list(waypoints)

    def current_pose_callback(self, msg):
        with self.lock:
            self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is None:
            return

        distance_to_goal = np.hypot(self.current_location.x - self.goal_point.x, self.current_location.y - self.goal_point.y)
        if distance_to_goal <= self.distance_to_goal_limit:
            rospy.loginfo("%s - Goal reached", rospy.get_name())
            self.publish_lane_from_waypoints_list([])
            self.goal_point = None

    def convert_laneletseq_to_waypoints_list(self, laneletseq):
        waypoints = []
        previous_point = None

        for j, lanelet in enumerate(laneletseq):
            speed_ref = lanelet.attributes['speed_ref'] if 'speed_ref' in lanelet.attributes else self.speed_limit
            speed = min(float(speed_ref), self.speed_limit) / 3.6
            target_spacing = max(0.5, min(2.0, speed * 0.5))

            for i, point in enumerate(lanelet.centerline):
                if i == 0 and j != 0:
                    continue

                if previous_point is None:
                    waypoints.append(self.create_waypoint(point.x, point.y, point.z, speed))
                    previous_point = point
                    continue

                distance = np.hypot(point.x - previous_point.x, point.y - previous_point.y)
                steps = max(1, int(np.ceil(distance / target_spacing)))

                for step in range(1, steps + 1):
                    ratio = step / steps
                    x = previous_point.x + ratio * (point.x - previous_point.x)
                    y = previous_point.y + ratio * (point.y - previous_point.y)
                    z = previous_point.z + ratio * (point.z - previous_point.z)
                    waypoints.append(self.create_waypoint(x, y, z, speed))

                previous_point = point

        if not waypoints:
            return waypoints

        closest_idx = min(
            range(len(waypoints)),
            key=lambda i: np.hypot(
                waypoints[i].position.x - self.goal_point.x,
                waypoints[i].position.y - self.goal_point.y
            )
        )

        waypoints = waypoints[:closest_idx + 1]
        self.goal_point = BasicPoint2d(waypoints[-1].position.x, waypoints[-1].position.y)
        self.apply_endpoint_speed_ease_out(waypoints)

        return waypoints

    def create_waypoint(self, x, y, z, speed):
        waypoint = Waypoint()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoint.speed = speed
        return waypoint

    def apply_endpoint_speed_ease_out(self, waypoints):
        path_xy = np.array([(wp.position.x, wp.position.y) for wp in waypoints])
        segment_lengths = np.sqrt(np.sum(np.diff(path_xy, axis=0)**2, axis=1))
        distances_from_start = np.insert(np.cumsum(segment_lengths), 0, 0)
        distances_to_endpoint = distances_from_start[-1] - distances_from_start
        speed_limits = np.sqrt(2 * self.default_deceleration * distances_to_endpoint)

        for waypoint, speed_limit in zip(waypoints, speed_limits):
            waypoint.speed = min(waypoint.speed, speed_limit)

    def publish_lane_from_waypoints_list(self, waypoints):
        lane = Path()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.global_path_pub.publish(lane)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('global_planner')
    node = GlobalPlanner()
    node.run()
