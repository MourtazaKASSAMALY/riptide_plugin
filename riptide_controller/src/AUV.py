#!/usr/bin/env python3

from roblib import *
import abc
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *


############################  Classes generic definition  ##################################


class AUV(object):
    __metaclass__ = abc.ABCMeta
    
    def __init__(self, initialState, reliefData = None):
        """Initialiser. Give here the initial state of your AUV."""
        
        self.X = initialState
        
        if reliefData is not None:
            self.reliefMatrix = reliefData[0]
            self.reliefRealSize = reliefData[1]

    @abc.abstractmethod
    def evolX(self, u):
        """Method that returns the derivative of the state vector, given the current state vector and the input commands."""
        pass
    
    def getEchosounder(self):
        """Simulate the sensor that gives the distance to the floor"""
        x = int((self.X[0,0])*np.shape(self.reliefMatrix)[0]/self.reliefRealSize[0])
        y = int((self.X[1,0])*np.shape(self.reliefMatrix)[1]/self.reliefRealSize[1])
        dist = self.X[2,0] - self.reliefMatrix[y,x]
        return dist

    def getMagnetometer(self):
        return (self.X[3,0], self.X[4,0], self.X[5,0])

    def getDepthmeter(self):
        return self.X[2,0]


###################################################################################
