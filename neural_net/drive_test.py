#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017

@author: jaerock
"""

import cv2
import numpy as np
#import keras
import sklearn
from progressbar import ProgressBar

#import resnet
import const
from net_model import NetModel
from drive_data import DriveData
from config import Config
from image_process import ImageProcess

config = Config.neural_net

###############################################################################
#
class DriveTest:
    
    ###########################################################################
    # model_path = 'path_to_pretrained_model_name' excluding '.h5' or 'json'
    # data_path = 'path_to_drive_data'  e.g. ../data/2017-09-22-10-12-34-56'
    def __init__(self, model_path, data_path):
        if data_path[-1] == '/':
            data_path = data_path[:-1]

        loc_slash = data_path.rfind('/')
        if loc_slash != -1: # there is '/' in the data path
            model_name = data_path[loc_slash+1:] # get folder name
            #model_name = model_name.strip('/')
        else:
            model_name = data_path
        csv_path = data_path + '/' + model_name + const.DATA_EXT   
        
        self.data = DriveData(csv_path)
        
        self.test_generator = None
        
        self.num_test_samples = 0        
        #self.config = Config()
        
        self.net_model = NetModel(model_path)
        self.net_model.load()
        
        self.image_process = ImageProcess()
        self.data_path = data_path

      
    ###########################################################################
    #
    def _prepare_data(self):
    
        self.data.read()
        
        samples = list(zip(self.data.image_names, self.data.measurements))

        self.test_data = samples
        
        self.num_test_samples = len(self.test_data)
        
        print('Test samples: ', self.num_test_samples)
    

    ###########################################################################
    #
    def _prep_generator(self):
        
        if self.data_path == None:
            raise NameError('data_path must be set.')
            
        def _prepare_batch_samples(batch_samples):
            images = []
            measurements = []

            for image_name, measurement in batch_samples:
                
                image_path = self.data_path + '/' + image_name
                image = cv2.imread(image_path)

                # if collected data is not cropped then crop here
                # otherwise do not crop.
                if Config.data_collection['crop'] is not True:
                    image = image[Config.data_collection['image_crop_y1']:Config.data_collection['image_crop_y2'],
                                  Config.data_collection['image_crop_x1']:Config.data_collection['image_crop_x2']]

                image = cv2.resize(image, 
                                    (config['input_image_width'],
                                    config['input_image_height']))
                image = self.image_process.process(image)
                images.append(image)

                steering_angle = measurement
                
                if abs(steering_angle) < config['steering_angle_jitter_tolerance']:
                    steering_angle = 0
                
                measurements.append(steering_angle*config['steering_angle_scale'])
                
                ## data augmentation <-- doesn't need since this is not training
                #append, image, steering_angle = _data_augmentation(image, steering_angle)
                #if append is True:
                #    images.append(image)
                #    measurements.append(steering_angle*config['steering_angle_scale'])

            return images, measurements
            
        def _generator(samples, batch_size=config['batch_size']):
            num_samples = len(samples)
            while True: # Loop forever so the generator never terminates
                
                bar = ProgressBar()
                
                samples = sklearn.utils.shuffle(samples)

                for offset in bar(range(0, num_samples, batch_size)):
                    batch_samples = samples[offset:offset+batch_size]

                    images, measurements = _prepare_batch_samples(batch_samples)
                    X_train = np.array(images).reshape(-1, 
                                        config['input_image_height'],
                                        config['input_image_width'],
                                        config['input_image_depth'])
                    y_train = np.array(measurements)
                    y_train = y_train.reshape(-1, 1)
                    
                    yield X_train, y_train

        self.test_generator = _generator(self.test_data)
        
    
    ###########################################################################
    #
    def _start_test(self):

        if (self.test_generator == None):
            raise NameError('Generators are not ready.')
        
        print('Evaluating the model with test data sets ...')
        ## Note: Do not use multiprocessing or more than 1 worker.
        ##       This will genereate threading error!!!
        score = self.net_model.model.evaluate_generator(self.test_generator, 
                                self.num_test_samples//config['batch_size']) 
                                #workers=1)
        print('Loss: {0}'.format(score)) #[0], "Accuracy: ", score[1])
        #print("\nLoss: ", score[0], "rmse: ", score[1])
        
    

   ###########################################################################
    #
    def test(self):
        self._prepare_data()
        self._prep_generator()
        self._start_test()
        Config.summary()

