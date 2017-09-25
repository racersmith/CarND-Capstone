#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
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

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.len_base_waypoints = None

        self.publisher()
        rospy.spin()


    def pose_cb(self, msg):
        pos = msg.pose.pose.position
        yaw = msg.pose.pose.orientation[2]

        if self.base_waypoints is not None:
            # TODO: Implement
            # find closest waypoint
            closest = self.distance(pos, self.base_waypoints[0].pose.pose.position)
            closest_index = 0
            for i in range(1, len(self.base_waypoints)):
                test_distance = self.distance(pos, self.base_waypoints[i])
                if test_distance < closest:
                    closest = test_distance
                    closest_index = i

            # from heading determine next waypoint
            closest_point = self.base_waypoints[closest_index].pose.pose.position
            angle_to_closest = math.atan2(closest_point.y - pos.y, closest_point.x - pos.x)
            heading = abs(yaw - angle_to_closest)
            if heading > math.pi/4:
                closest_index += 1

            # Publish final waypoints
            final_waypoints = []
            for i in range(closest_index, closest_index + LOOKAHEAD_WPS):
                final_waypoints.append(self.base_waypoints[i%self.len_base_waypoints])
            self.final_waypoints_pub.publish(final_waypoints)
            pass


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # base_waypoints is only published once on startup.
        # We need to capture and store this info for later.

        self.base_waypoints = waypoints.waypoints
        self.len_base_waypoints = len(self.base_waypoints)
        pass


    def traffic_cb(self, msg):
        if self.final_waypoints is not None:
            # TODO: Callback for /traffic_waypoint message. Implement
            pass


    def obstacle_cb(self, msg):
        if self.final_waypoints is not None:
            # TODO: Callback for /obstacle_waypoint message. We will implement it later
            pass


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


    def cumulative_distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.distance(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def publisher(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.final_waypoints_pub.publish(self.final_waypoints)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
