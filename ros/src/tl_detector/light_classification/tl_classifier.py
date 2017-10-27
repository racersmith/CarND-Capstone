from styx_msgs.msg import TrafficLight
from keras.models import load_model
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #load classifier
        self.model = load_model('trained_model.h5')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_state = np.argmax(self.model.predict(np.array([image])))

        if light_state == 0:
            return TrafficLight.RED
        if light_state == 1:
            return TrafficLight.YELLOW
        if light_state == 2:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKOWN
