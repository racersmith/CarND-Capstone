#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32


import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number

# Acceleration limit to determine if the car can stop for the light
# This serves as a quick calculation and does not use JMT.
# Accleration may actually exceed this value by some amount.
RED_LIGHT_MAX_ACCEL = -9.81

# Target acceleration for velocity changes when situation allows
TARGET_ACCEL = 9.81*0.05


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        # Parameters
        self.speed_limit = float(self.kmph2mps(rospy.get_param('waypoint_loader/velocity')))

        self.base_waypoints = None
        self.base_s = [0]
        self.n_base_waypoints = None
        self.traffic_index = None

        # Current State
        self.pose = None
        self.pos = None
        self.yaw = None
        self.vel = None

        self.last_waypoint_index = None

        # Process information and publish next waypoints
        # Endless loop while ros is running
        self.update_waypoints()
        rospy.spin()

    # Callback method for current pose subscription
    def pose_cb(self, msg):
        self.pose = msg
        self.pos = msg.pose.position

        # convert from quaternion to euler
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    # Callback method for velocity subscription
    def velocity_cb(self, msg):
        self.vel = msg.twist.linear.x

    # Callback method for waypoint subscription
    def waypoints_cb(self, waypoints):
        # base_waypoints is only published once on startup.
        # We need to capture and store this info for later.
        if self.base_waypoints is None:
            self.base_waypoints = waypoints.waypoints
            self.n_base_waypoints = len(self.base_waypoints)
            for i in range(1, self.n_base_waypoints):
                dist = self.distance(self.base_waypoints[i-1].pose.pose.position,
                                     self.base_waypoints[i].pose.pose.position)
                self.base_s.append(self.base_s[-1] + dist)

            rospy.loginfo('%s Base Waypoints Loaded', self.n_base_waypoints)
            rospy.loginfo('Track Length: %s', max(self.base_s)-min(self.base_s))

    # Callback method for traffic light detection subscription
    def traffic_cb(self, msg):
        # At what future waypoint index is a red light?
        self.traffic_index = msg.data-5

    # Callback method for obstacle detection subscription
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Get velocity from single waypoint
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # Set velocity of single waypoint
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # Euclidean distance between two points in 3D space
    def distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    # Cumulative distance between series of waypoints
    def cumulative_distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.distance(waypoints[wp1].pose.pose.position,
                                  waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # Search for the closest base waypoint to the current pose
    # Start search at
    def find_closest_base_waypoint(self, starting_index, n_points):
        closest_index = starting_index % self.n_base_waypoints
        closest = self.distance(self.pos,
                                self.base_waypoints[closest_index].pose.pose.position)
        for i in range(1, n_points):
            test_index = (starting_index+i) % self.n_base_waypoints
            test_distance = self.distance(self.pos,
                                          self.base_waypoints[test_index].pose.pose.position)
            if test_distance < closest:
                closest = test_distance
                closest_index = test_index

        # Recursively search if the closest was found at an endpoint
        if n_points < self.n_base_waypoints:
            # closest was the first point
            if closest_index == starting_index:
                starting_index -= (n_points - 1 - 1)
                while starting_index < 0:
                    starting_index += self.n_base_waypoints
                closest_index = self.find_closest_base_waypoint(starting_index, n_points)

            # closest was last point
            elif closest_index == starting_index + n_points - 1:
                starting_index += (n_points - 1 - 1)
                closest_index = self.find_closest_base_waypoint(starting_index, n_points)

        return closest_index

    # Determine if the closest waypoint is in front or behind pose
    # Output the next waypoint in front of pose
    def find_next_base_waypoint(self, closest_index):
        # from heading determine next waypoint
        closest_point = self.base_waypoints[closest_index].pose.pose.position
        angle_to_closest = math.atan2(closest_point.y - self.pos.y, closest_point.x - self.pos.x)
        heading_diff = abs(self.yaw - angle_to_closest)
        if heading_diff > math.pi / 4:
            closest_index += 1
        return closest_index

    @staticmethod
    def kmph2mps(velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    @staticmethod
    def distance_to_speed(v0, vt, a):
        if abs(a) < 0.01:
            return 100
        t = (vt - v0) / a
        return 1 / 2 * a * t ** 2 + v0 * t

    @staticmethod
    def target_accel(v0, vt, d):
        if abs(d) < 0.1:
            if v0 > vt:
                return 1
            else:
                return -1
        return (vt ** 2 - v0 ** 2) / (2 * d)

    @staticmethod
    def target_velocity(d, a, v0):
        x = 2 * a * d + v0 ** 2
        if x < 0:
            return 0
        else:
            return math.sqrt(x)

    # Control speed
    def set_speed(self, next_index, waypoints):
        s0 = self.base_s[next_index - 1]

        # Do we have a red light?
        if self.traffic_index > 0:
            if next_index > len(self.base_s):
                next_index = next_index%len(self.base_s)
                distance_to_light = self.base_s[self.traffic_index] - self.base_s[next_index] + self.base_s[-1]
            else:
                distance_to_light = self.base_s[self.traffic_index] - self.base_s[next_index]
            stop_accel = self.target_accel(self.vel, 0, distance_to_light)
            # Should we react?
            if RED_LIGHT_MAX_ACCEL < stop_accel <= -TARGET_ACCEL:
                # Set waypoint speeds
                for i in range(len(waypoints)):
                    d = self.base_s[next_index+i] - s0
                    if d > distance_to_light:
                        v = 0
                    else:
                        v = self.target_velocity(d, stop_accel, self.vel)
                    self.set_waypoint_velocity(waypoints, i, v)

        # Are we going the speed limit?
        elif self.vel != self.speed_limit:
            # Set waypoint speeds
            for i in range(len(waypoints)):
                # d = self.base_s[next_index+i] - s0
                # v = self.target_velocity(d, TARGET_ACCEL, self.vel)
                # v = min(v, self.speed_limit)
                v = self.speed_limit
                self.set_waypoint_velocity(waypoints, i, v)

        return waypoints

    # Update waypoints and publish
    def update_waypoints(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            if self.base_waypoints is not None and self.pose is not None:
                search_index = 0
                search_length = self.n_base_waypoints

                # If we have some idea where we are then do a more focused search
                if self.last_waypoint_index is not None:
                    search_index = self.last_waypoint_index - LOOKAHEAD_WPS
                    while search_index < 0:
                        search_index += self.n_base_waypoints
                    search_length = 2*LOOKAHEAD_WPS

                closest_waypoint = self.find_closest_base_waypoint(search_index, search_length)
                self.car_index_pub.publish(closest_waypoint)
                next_waypoint = self.find_next_base_waypoint(closest_waypoint)

                # Construct final waypoints
                final_waypoints = []
                for i in range(next_waypoint, next_waypoint + LOOKAHEAD_WPS):
                    final_waypoints.append(self.base_waypoints[i % self.n_base_waypoints])

                # Adjust the speed of the waypoints based on traffic lights
                final_waypoints = self.set_speed(next_waypoint, final_waypoints)

                # Construct and publish message
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane)
                self.last_waypoint_index = next_waypoint

            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
