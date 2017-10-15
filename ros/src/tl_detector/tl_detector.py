#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.camera_image = None
        self.lights = []
        self.has_image = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/car_index', Int32, self.car_index_cb)
        sub5 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.car_index = None
        self.next_waypoints = None
        self.stop_map = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    def waypoints_cb(self, waypoints):
        if self.base_waypoints is None:
            self.base_waypoints = waypoints.waypoints

            self.stop_map = []
            for x, y in self.config['stop_line_positions']:
                stop_point = self.Point(x, y)
                self.stop_map.append(self.get_closest_waypoint(stop_point))
                rospy.loginfo("Stop Index: {}".format(self.stop_map[-1]))

    def car_index_cb(self, msg):
        self.car_index = msg

    def final_waypoints_cb(self, msg):
        self.next_waypoints = msg

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # Find next light
        # Determine distance
        # Set stop waypoint
        for i, light in enumerate(msg.lights):
            dist = math.sqrt(self.squared_error_2d(self.pose.pose.position, light.pose.pose.position))
            state = light.state
            # rospy.loginfo("Traffic Light {}: dist={:4.2f}, state={}".format(i, dist, state))

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        rospy.loginfo("image cb")

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pos):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_index = None
        closest_se = None
        for i, waypoint in enumerate(self.base_waypoints):
            error = self.squared_error_2d(pos, waypoint.pose.pose.position)
            if closest_se is None or error < closest_se:
                closest_se = error
                closest_index = i

        return closest_index

    def squared_error_2d(self, a, b):
        return (a.x - b.x)**2 + (a.y - b.y)**2

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use transform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        if trans is not None:
            euler = tf.transformations.euler_from_quaternion(rot)
            camera_matrix = [[fx, 0, image_width/2],[0, fy, image_height/2],[0,0,1]]
            x, y = cv2.projectPoints([point_in_world.x, point_in_world.y], euler, trans, camera_matrix)


        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)
        rospy.loginfo("x: {:4.2f}, y: {:4.2f}".format(x, y))
        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        next_stop_index = -1
        traffic_index = None
        if(self.car_index is not None and self.stop_map is not None):
            # TODO find the closest visible traffic light (if one exists)
            # Next Light
            next_stop_index = self.stop_map[0]
            traffic_index = 0
            for i, stop_index in enumerate(self.stop_map):
                if self.car_index > next_stop_index and stop_index >= self.car_index:
                    next_stop_index = stop_index
                    traffic_index = i

            dist = math.sqrt(self.squared_error_2d(self.lights[traffic_index].pose.pose.position), self.pose)
            rospy.loginfo("Traffic Light {}: dist={:4.2f}, state={}".format(traffic_index, dist, self.lights[traffic_index].state))

        if self.lights is not None and traffic_index is not None:
            light = self.lights[traffic_index]
            state = self.get_light_state(light)
            return next_stop_index, state
        return next_stop_index, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')