#!/usr/bin/env python

from ros_compatible_node import (
    CompatibleNode,
    ros_ok,
    ros_shutdown,
    ros_on_shutdown,
    ros_timestamp,
    QoSProfile,
    latch_on,
    ros_init,
    get_service_response,
    ROS_VERSION)

import rospy
import sys
import math
import os

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from rosgraph_msgs.msg import Clock

import numpy as np
import cv2

import AVISEngine

ROS_VERSION = 1

class AVISEngineROSBridge(CompatibleNode):

    """
    AVIS Engine Ros bridge
    """

    VEHICLE_CONTROL_TIMEOUT = 1.

    def __init__(self, car, rospy_init=True, executor=None):
        """
        Constructor

        """
        super(AVISEngineROSBridge, self).__init__("ros_bridge_node", car, rospy_init=rospy_init)
        self.executor = executor
        self.car = car
        self.counter = 0
        self.image_publisher = rospy.Publisher("image_raw",Image)
        self.bridge = CvBridge()

    # pylint: disable=attribute-defined-outside-init
    def initialize_bridge(self, params):
        """
        Initialize the bridge
        """
        ros_on_shutdown(self.destroy)

        self.parameters = params
        

        if ROS_VERSION == 1:
            self.ros_timestamp = 0
            self.callback_group = None

        # if not self.parameters["passive"]:
          
        self.loginfo("Parameters:")
        for key in self.parameters:
            self.loginfo("  {}: {}".format(key, self.parameters[key]))

        # Communication topics
        self.clock_publisher = self.new_publisher(Clock, 'clock')
        # self.image_publisher = self.new_publisher(CompressedImage, 'image_raw')

    def _update(self, frame_id, timestamp):
        """
        update all
        :return:
        """
        self.world_info.update(frame_id, timestamp)


    def update_clock(self, AVIS_timestamp):
        """
        perform the update of the clock
        :return:
        """
        if ros_ok():
            self.ros_timestamp = ros_timestamp(AVIS_timestamp.elapsed_seconds, from_sec=True)
            self.clock_publisher.publish(Clock(clock=self.ros_timestamp))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        self.loginfo("Shutting down...")
        ## TODO : Add tasks to do
        self.loginfo("Object update finished.")

        super(AVISEngineROSBridge, self).destroy()

    def routine(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.counter = self.counter + 1

            #Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
            self.car.setSpeed(20)

            #Set the Steering of the car -10 degree from center
            self.car.setSteering(-10)

            #Get the data. Need to call it every time getting image and sensor data
            self.car.getData()

            #Start getting image and sensor data after 4 loops. for unclear some reason it's really important 
            if(self.counter > 4):
                #returns a list with three items which the 1st one is Left sensor data, the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
                sensors = self.car.getSensors() 
                #EX) sensors[0] returns an int for left sensor data in cm

                #returns an opencv image type array. if you use PIL you need to invert the color channels.
                image = self.car.getImage()

                #returns an integer which is the real time car speed in KMH
                carSpeed = self.car.getSpeed()

                try:
                    self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                    
                except CvBridgeError as e:
                    print(e)


def main(args=None):
    executor = None
    parameters = {}
    AVIS_Client = AVISEngine.car()
    if ROS_VERSION == 1:
        AVIS_Bridge = AVISEngineROSBridge(AVIS_Client)

    parameters['host'] = AVIS_Bridge.get_param('host', '127.0.0.1')
    parameters['port'] = AVIS_Bridge.get_param('port', 25001)

    AVIS_Bridge.loginfo("Trying to connect to {host}:{port}".format(
        host=parameters['host'], port=parameters['port']))
    
    try:
        
        AVIS_Client.connect(parameters['host'], parameters['port'])

        AVIS_Bridge.initialize_bridge(parameters)
     
        AVIS_Bridge.routine()

        if ROS_VERSION == 1:
            rospy.spin()
        elif ROS_VERSION == 2:
            executor.spin()
    except (IOError, RuntimeError) as e:
        AVIS_Bridge.logerr("Error: {}".format(e))
    except KeyboardInterrupt:
        pass
    finally:
        ros_shutdown()

if __name__ == "__main__":
    main()