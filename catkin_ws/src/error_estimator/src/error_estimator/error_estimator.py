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

        # Drone controller topic publisher
        self.control_var_pub = rospy.Publisher(
                "error_estimator/control_value",
                std_msgs.msg.Float64MultiArray,
                queue_size=1)
        return

    def kalman_callback(self, data):
        """
        Calculate and send the control variable of the drones

        The control is done with a P controller fed with the actual
        distances between the drones, which are obtained in the Kalman
        node.

        It will not work until the desired distances are obtained from
        the pattern generator.
        """
        if not self.start:
            return
        self.predicted_distances = array_operations.multiarray2np(data)
        self.errors = error_functions.simple_differences(
                self.desired_distances, self.predicted_distances)

        # P gain
        Kp = 1.2
        units = np.zeros([1,2])

        drone = 2 #must be changed to fit with actual drone

        for j in range(0, len(self.errors)):
            if(self.errors[drone-1,j] != 0.0):
            # This is the row of the drone i am

                # Add up the unit vector of all in this row
                unit_vector = (self.desired_distances[drone-1,j]
                               / dist(self.desired_distances[drone-1,j]))
                if drone-1 > j:
                    units -= unit_vector * self.errors[drone-1,j]
                elif drone-1 < j:
                    units += unit_vector * self.errors[drone-1,j]

        self.control_var_pub.publish(array_operations.np2multiarray(Kp*units))
        return

    def pat_gen_start_callback(self, data):
        """
        Store the desired distances, and allow the controller to start
        """
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
