from styx_msgs.msg import TrafficLight
from keras.models import load_model
import numpy as np
import os

class TLClassifier(object):
    def __init__(self):
        #load classifier
        print("TLClassifier dir: ", os.getcwd())
        self.model = load_model('light_classification/trained_model.h5')
        print(self.model.summary())
        # Test model
        dummy_image = np.zeros((24,24,3))
        print(self.model.predict(np.array([dummy_image])))
        print(self.get_classification(dummy_image))

        # self.model._make_predict_function()

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
