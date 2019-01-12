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


class ErrorEstimatorNode():

    def __init__(self):
        self.start = False
        # Errors matrix
        self.errors = np.array([[0, 1, 2],[1, 0, 3],[2, 3, 0]])
        self.predicted_distances = np.array()
        self.desired_distances = np.array()

        # Kalman filter topic susbscribers
        rospy.Subscriber("kalman/state", std_msgs.msg.Float64MultiArray,
                         self.kalman_callback, queue_size=1)
        # Pattern generator topic susbscribers
        rospy.Subscriber("pattern_generator/start",
                         std_msgs.msg.Float64MultiArray,
                         self.pat_gen_start_callback, queue_size=1)

        # Drone controller topic publishers
        self.errors_pub = rospy.Publisher(
                "error_estimator/errors",
                std_msgs.msg.Float64MultiArray,
                queue_size=1)
        return

    def kalman_callback(self, data):
        self.predicted_distances = array_operations.multiarray2np(data)
        self.errors = error_functions.simple_differences(
                self.desired_distances, self.predicted_distances)
        self.errors_pub.publish(array_operations.np2multiarray(self.errors))
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
