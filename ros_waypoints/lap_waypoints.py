#!/usr/bin/env python3
"""Publish a route window in lap order from the measured rear-axle position."""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from progress import APPROACH_DECEL, STOP_DISTANCE, RouteProgress


class LapWaypoints:
    def __init__(self):
        route = np.loadtxt(rospy.get_param('~route'), delimiter=',')
        self.cruise_speed = rospy.get_param('~speed') / 3.6
        self.progress = RouteProgress(route[:, :2])
        self.waypoints = []
        for x, y, yaw in route:
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = x
            waypoint.pose.pose.position.y = y
            waypoint.pose.pose.orientation.z = math.sin(yaw / 2)
            waypoint.pose.pose.orientation.w = math.cos(yaw / 2)
            waypoint.forward = True
            self.waypoints.append(waypoint)
        self.publisher = rospy.Publisher('/final_waypoints', Lane, queue_size=1, tcp_nodelay=True)
        self.subscriber = rospy.Subscriber('/smart/rear_pose', PoseStamped, self.update,
                                           queue_size=1, tcp_nodelay=True)

    def update(self, pose):
        position = pose.pose.position
        station, _, segment, fraction = self.progress.update(position.x, position.y)
        # Keep a segment available for Stanley even when the endpoint is closest.
        first = min(segment + int(fraction >= 0.5), len(self.waypoints) - 2)
        remaining = self.progress.stations[-1] - station
        target = (min(self.cruise_speed, math.sqrt(2 * APPROACH_DECEL * remaining))
                  if remaining > STOP_DISTANCE else 0.0)
        lane = Lane(header=pose.header, waypoints=self.waypoints[first:first + 20])
        for waypoint in lane.waypoints:
            waypoint.twist.twist.linear.x = target
        self.publisher.publish(lane)


if __name__ == '__main__':
    rospy.init_node('lap_waypoints')
    lap = LapWaypoints()
    rospy.spin()
