#!/usr/bin/env python3
"""
Error estimator top level module
"""
# Standard libraries
# Third-party libraries
import numpy as np
import rospy
import geometry_msgs.msg
import std_msgs.msg
# Local libraries
from error_estimator import array_operations
from error_estimator import error_functions

def dist(vector):
    length = np.sqrt(pow(vector[0],2)+pow(vector[1],2))
    return length

class ErrorEstimatorNode():

    def __init__(self):
        self.start = False
        # Errors matrix
        self.errors = np.array([[0, 1, 2],[1, 0, 3],[2, 3, 0]])
        self.predicted_distances = np.array([])
        self.desired_distances = np.array([])

        # Kalman filter topic susbscribers
        rospy.Subscriber("kalman/state", std_msgs.msg.Float64MultiArray,
                         self.kalman_callback, queue_size=1)
        # Pattern generator topic susbscribers
        rospy.Subscriber("pattern_generator/start",
                         std_msgs.msg.Float64MultiArray,
                         self.pat_gen_start_callback, queue_size=1)

        # Drone controller topic publishers
        #self.errors_pub = rospy.Publisher(
         #       "error_estimator/errors",
          #      std_msgs.msg.Float64MultiArray,
           #     queue_size=1)
        self.control_variable_pub = rospy.Publisher(
                "error_estimator/control_value",
                std_msgs.msg.Float64MultiArray,
                queue_size=1)
        
        return

    def kalman_callback(self, data):
        self.predicted_distances = array_operations.multiarray2np(data)
        self.errors = error_functions.simple_differences(
                self.desired_distances, self.predicted_distances)
        

        #kalman P gain
        Kp = 1.2
        units = np.zeros([1,2])

        drone = 2 #must be changed to fit with actual drone

        for j in range(0, len(self.errors)):
            if(self.errors[drone-1,j] != 0.0):
            #this is the row of the drone i am

                #add op the unit vector*e of all in this row
                if drone-1 > j:
                    units = units - (self.desired_distances[drone-1,j]/dist(self.desired_distances[drone-1,j]))*self.errors[drone-1,j]
                elif drone-1 < j:
                    units = units + (self.desired_distances[drone-1,j]/dist(self.desired_distances[drone-1,j]))*self.errors[drone-1,j]

        self.control_variable_pub.publish(array_operations.np2multiarray(Kp*units))
        #self.errors_pub.publish(array_operations.np2multiarray(self.errors))
        return

    def pat_gen_start_callback(self, data):
        self.desired_distances = array_operations.multiarray2np(data.data)
        self.start = True
        return

    def run(self):
        while not self.start:
            rospy.sleep(1)
        rospy.spin()
        return


def main():
    # Instantiate the error_estimator node class and run it
    error_estimator = ErrorEstimatorNode()
    error_estimator.run()
    return
