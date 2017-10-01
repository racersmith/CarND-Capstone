#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from styx_msgs.msg import Lane, Waypoint


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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

# Acceleration limit to determine if the car can stop for the light
# This serves as a quick calculation and does not use JMT.
# Accleration may actually exceed this value by some amount.
RED_LIGHT_MAX_ACCEL = 9.81

# Target acceleration for velocity changes when situation allows
# Again this is used for a quick calculation that will feed JMT.
TARGET_ACCEL = 9.81*0.1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        # rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.base_s = [0]
        self.n_base_waypoints = None
        self.traffic_index = None

        # Current State
        self.pose = None
        self.pos = None
        self.yaw = None
        self.vel = None

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
        self.traffic_index = msg

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

    # Update waypoints and publish
    def update_waypoints(self):
        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            if self.base_waypoints is not None and self.pose is not None:
                # find closest waypoint
                closest = self.distance(self.pos,
                                        self.base_waypoints[0].pose.pose.position)
                closest_index = 0
                for i in range(1, len(self.base_waypoints)):
                    test_distance = self.distance(self.pos,
                                                  self.base_waypoints[i].pose.pose.position)
                    if test_distance < closest:
                        closest = test_distance
                        closest_index = i

                # from heading determine next waypoint
                closest_point = self.base_waypoints[closest_index].pose.pose.position
                angle_to_closest = math.atan2(closest_point.y - self.pos.y, closest_point.x - self.pos.x)
                heading_diff = abs(self.yaw - angle_to_closest)
                if heading_diff > math.pi / 4:
                    closest_index += 1

                # Publish final waypoints
                final_waypoints = []
                for i in range(closest_index, closest_index + LOOKAHEAD_WPS):
                    final_waypoints.append(self.base_waypoints[i % self.n_base_waypoints])

                # Construct and publish message
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
