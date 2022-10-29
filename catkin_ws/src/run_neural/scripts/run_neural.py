#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""

import threading 
import cv2
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import math

import sys
import os

import const
from image_converter import ImageConverter
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# if Config.data_collection['vehicle_name'] == 'fusion':
#     from fusion.msg import Control
# elif Config.data_collection['vehicle_name'] == 'rover':
#     from geometry_msgs.msg import Twist
#     from rover.msg import Control
# else:
#     exit(Config.data_collection['vehicle_name'] + 'not supported vehicle.')


config = Config.neural_net
velocity = 0.0
class NeuralControl:
    def __init__(self, weight_file_name, base_weight_name=None):
        rospy.init_node('run_neural')
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.rate = rospy.Rate(30)
        self.drive= DriveRun(weight_file_name, base_weight_name)
        rospy.Subscriber(Config.data_collection['camera_image_topic'], Image, self._controller_cb)
        self.image = None
        self.image_processed = False
        #self.config = Config()
        self.braking = False
        self.lstm_image = []
        self.lstm_vel = []
        self.term_count = 0
        self.image_name = 0
    def _controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        cropped = img[Config.data_collection['image_crop_y1']:Config.data_collection['image_crop_y2'],
                      Config.data_collection['image_crop_x1']:Config.data_collection['image_crop_x2']]
                      
        img = cv2.resize(cropped, (config['input_image_width'],
                                   config['input_image_height']))
        cv2.imwrite('/home/eunseo/aug/'+str(self.image_name)+'.png', img)
        self.image = self.image_process.process(img)
        # print(img.shape)
        ## this is for CNN-LSTM net models
        if config['lstm'] is True:
            if self.term_count % Config.run_neural['lstm_dataterm'] is 0:
                self.lstm_image.append(self.image)
                if len(self.lstm_image) > config['lstm_timestep'] :
                    del self.lstm_image[0]
                if config['num_inputs'] == 2:
                    self.lstm_vel.append(velocity)
                    if len(self.lstm_vel) > config['lstm_timestep']:
                        del self.lstm_vel[0]
            self.term_count += 1
                    
        self.image_processed = True
        self.image_name += 1
        
    def _timer_cb(self):
        self.braking = False

    def apply_brake(self):
        self.braking = True
        timer = threading.Timer(Config.run_neural['brake_apply_sec'], self._timer_cb) 
        timer.start()

      
def pos_vel_cb(value):
    global velocity

    vel_x = value.twist.twist.linear.x 
    vel_y = value.twist.twist.linear.y
    vel_z = value.twist.twist.linear.z
    
    velocity = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        
def main(weight_file_name, base_weight_name=None):

    # ready for neural network
    neural_control = NeuralControl(weight_file_name, base_weight_name)
    
    rospy.Subscriber(Config.data_collection['base_pose_topic'], Odometry, pos_vel_cb)
    # ready for /bolt topic publisher
    # joy_pub = rospy.Publisher(Config.data_collection['vehicle_control_topic'], Twist, queue_size = 10)
    joy_data = Twist()

    # if Config.data_collection['vehicle_name'] == 'rover':
    joy_pub4mavros = rospy.Publisher('/cmd_vel', Twist, queue_size=20)

    print('\nStart running. Grrrrrrr Kak Kak Baaaaam!!!!......')
    print('steer \tthrt: \tbrake \tvelocity \tHz')

    use_predicted_throttle = True if config['num_outputs'] == 2 else False
    while not rospy.is_shutdown():
        if neural_control.image_processed is False:
            continue
        prediction = neural_control.drive.run((neural_control.image, ))
        joy_data = Twist()
        joy_data.linear.x = 0.06
        joy_data.angular.z = prediction

        joy_pub4mavros.publish(joy_data)


        ## print out
        # print(joy_data.steer, joy_data.throttle, joy_data.brake, velocity)
        cur_output = '{0:.3f} \r'.format(joy_data.angular.z)

        sys.stdout.write(cur_output)
        sys.stdout.flush()
            
        
        ## ready for processing a new input image
        neural_control.image_processed = False
        neural_control.rate.sleep()



if __name__ == "__main__":
    try:
        if config['style_run'] is True:
            if len(sys.argv) != 3:
                exit('Usage:\n$ rosrun run_neural run_neural.py style_weight_name base_weight_name')
            main(sys.argv[1], sys.argv[2])
                
        else:
            if len(sys.argv) != 2:
                exit('Usage:\n$ rosrun run_neural run_neural.py weight_file_name')
            main(sys.argv[1])

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
        
