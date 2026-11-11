#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from styx_msgs.msg import Lane
from tf.transformations import euler_from_quaternion


MIN_SPEED = 0.5  # m/s, keeps the steering correction finite at startup


class Stanley:
    def __init__(self):
        self.gain = rospy.get_param('~gain', 1.0)
        self.wheelbase = rospy.get_param('~wheelbase', 1.868)
        self.rear_pose = None
        self.speed = None
        self.target_speed = None
        self.segments = []

        self.cmd_pub = rospy.Publisher('/smart/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/smart/rear_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/smart/velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.path_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.target_speed_cb, queue_size=1)

    def pose_cb(self, msg):
        self.rear_pose = msg.pose

    def velocity_cb(self, msg):
        velocity = msg.twist.linear
        self.speed = math.hypot(velocity.x, velocity.y)

    def path_cb(self, lane):
        points = [waypoint.pose.pose.position for waypoint in lane.waypoints]
        segments = []
        for start, end in zip(points, points[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            length = math.hypot(dx, dy)
            if length > 0.0:
                segments.append((start.x, start.y, dx / length, dy / length, length))
        self.segments = segments

    def target_speed_cb(self, lane):
        self.target_speed = lane.waypoints[0].twist.twist.linear.x

    def steering_angle(self, pose, speed, segments):
        q = pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        front_x = pose.position.x + self.wheelbase * math.cos(yaw)
        front_y = pose.position.y + self.wheelbase * math.sin(yaw)

        nearest_distance = math.inf
        for start_x, start_y, tangent_x, tangent_y, length in segments:
            offset_x = front_x - start_x
            offset_y = front_y - start_y
            along = offset_x * tangent_x + offset_y * tangent_y
            along = max(0.0, min(length, along))
            distance = (offset_x - along * tangent_x)**2 + (offset_y - along * tangent_y)**2
            if distance < nearest_distance:
                nearest_distance = distance
                path_yaw = math.atan2(tangent_y, tangent_x)
                # A front axle to the right of the path needs positive (left) steering.
                cross_track_error = offset_x * tangent_y - offset_y * tangent_x

        heading_error = math.remainder(path_yaw - yaw, math.tau)
        correction = math.atan2(self.gain * cross_track_error, max(speed, MIN_SPEED))
        return heading_error + correction

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pose = self.rear_pose
            speed = self.speed
            target_speed = self.target_speed
            segments = self.segments
            if pose is not None and speed is not None and target_speed is not None and segments:
                cmd = Twist()
                cmd.linear.x = target_speed
                if target_speed > 0.0:
                    # The Gazebo adapter interprets angular.z as a steering angle.
                    cmd.angular.z = self.steering_angle(pose, speed, segments)
                self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('stanley')
    try:
        Stanley().run()
    except rospy.ROSInterruptException:
        pass
