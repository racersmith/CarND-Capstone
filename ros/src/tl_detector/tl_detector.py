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
import numpy as np

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
        self.car_index = msg.data

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

    def project_with_fov(self, point):
        # Point in space to map to image plane
        # This point should be relative to the camera
        d = point.pose.position.x
        x = point.pose.position.y
        y = point.pose.position.z-1

        # Camera characteristics
        fov_x = self.config['camera_info']['focal_length_x']
        fov_y = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # This the half height/width of the image when projected
        # to the depth of the point
        # normalizer_x = d * math.tan(fov_x/2.0)
        # normalizer_y = d * math.tan(fov_y/2.0)

        # Position in image using upper left corner as origin
        # img_x = image_width/2.0 + image_width/2.0 * x/normalizer_x
        # img_y = image_height/2.0 + image_height/2.0 * y/normalizer_y

        # img_y = math.atan(y/d)/fov_y*image_height
        img_x = 2650*x/d
        img_y = 2250*y/d

        # Position in image using image center as origin
        # img_x = image_width * x / normalizer_x
        # img_y = image_height * y / normalizer_y

        return int(img_x), int(img_y)

    def project_to_image_plane(self, light):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
        obj_pos = light.pose.pose.position

        fx = self.config['camera_info']['focal_length_x']

        # get transform between pose of camera and world frame
        # trans = None
        # rot = None
        base_light = None

        try:
            # now = rospy.Time.now()
            # self.listener.waitForTransform("/base_link",
            #       "/world", now, rospy.Duration(1.0))
            # (trans, rot) = self.listener.lookupTransform("/base_link",
            #       "/world", now)

            # Transform pose of light relative to car
            # base_light = PoseStamped()
            base_light = self.listener.transformPose("base_link", light.pose)
            rospy.loginfo("Light relative to car ({:4.2f}, {:4.2f}, {:4.2f})".format(base_light.pose.position.x,
                                                                                     base_light.pose.position.y,
                                                                                     base_light.pose.position.z))
            # rospy.loginfo(base_light)
            # rospy.loginfo(euler)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use transform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        # if trans is not None:
        #     obj_points = np.float32([[obj_pos.x, obj_pos.y, obj_pos.z]]).reshape(-1, 3)
        #     # euler = tf.transformations.euler_from_quaternion(rot)
        #     camera_matrix = np.array([[fx, 0, image_width/2],
        #                               [0, fy, image_height/2],
        #                               [0, 0, 1]])
        #     dist_coef = np.zeros(4)
        #
        #     # Map between car coord to cv2
        #     # rospy.loginfo(trans)
        #     # rospy.loginfo(euler)
        #     # trans = np.array([trans[1], trans[2], trans[0]])
        #     # euler = np.array([euler[1], euler[2], euler[0]])
        #
        #     img_points, _ = cv2.projectPoints(obj_points, euler, trans, camera_matrix, dist_coef)
        #     x = img_points[0][0][0]
        #     y = img_points[0][0][1]

        if base_light is not None:
            # Simulator uses FOV
            if fx < 100:
                x, y = self.project_with_fov(base_light)
            # Real car uses focal length
            else:
                rospy.loginfo('Real car detected...  Process image using focal length!')


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

        obj_pos = light.pose.pose.position
        x, y = self.project_to_image_plane(light)
        rospy.loginfo("Image Position: {}, {}".format(x, y))
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
        if self.car_index is not None and self.stop_map is not None:
            # TODO find the closest visible traffic light (if one exists)
            # Next Light
            next_stop_index = self.stop_map[0]
            traffic_index = 0
            for i, stop_index in enumerate(self.stop_map):
                if stop_index >= self.car_index > next_stop_index:
                    next_stop_index = stop_index
                    traffic_index = i

            dist = math.sqrt(self.squared_error_2d(self.lights[traffic_index].pose.pose.position, self.pose.pose.position))
            # rospy.loginfo("Traffic Light {}: dist={:4.2f}, state={}".format(traffic_index,
            #                                                                 dist,
            #                                                                 self.lights[traffic_index].state))
            rospy.loginfo("Current: {}, Light {} @ {} in {}m".format(self.car_index,
                                                                     traffic_index,
                                                                     next_stop_index,
                                                                     dist))

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