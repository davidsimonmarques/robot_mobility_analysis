#!/usr/bin/env python

import numpy as np


class MovingAverageFilter:
    """
    This class will calculate the linear velocity and angular acceleration 
    based on the IMU data
    """

    def __init__(self,n_points,VEC_DIMENSION):     
        """
        Initalize object
        """
        self.data = np.zeros((n_points,VEC_DIMENSION))
        self.average = np.zeros((VEC_DIMENSION,1))



    def shift_data(self,data):
        new_data = np.copy(data)
        for i in range(len(data)):
            new_data[i] = data[i-1]
        data = new_data
        return data
    
    def test(self):
        average = np.zeros((2,1))
        vec = np.array([[1.0,1.0],[2.0,2.0],[3.0,3.0],[4.0,4.0]])
        vec = self.shift_data(vec)
        vec[0] = [9,9]
        for i in range(len(vec[0])):
                average[i] = np.divide(np.sum(vec[:,i]),len(vec))
        #average[0] = np.divide(np.sum(vec[:,0]),len(vec))
        #average = np.average(vec)
        
    
    def apply_moving_average_filter(self,input):
        """
        Function that performs calculations and apply the moving average filter
        """
        self.data = self.shift_data(self.data)
        self.data[0] = input
        for i in range(len(self.data[0])):
                self.average[i] = np.divide(np.sum(self.data[:,i]),len(self.data))
        
        return self.average