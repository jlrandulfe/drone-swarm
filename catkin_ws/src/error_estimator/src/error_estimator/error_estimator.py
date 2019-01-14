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
        self.predicted_rel_positions = np.array([])
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
        # Can not go through the routine without the desired distance
        # been given beforehand.
        if not self.start:
            return
        # Get the relative positions from the Kalman node. With them, calculate
        # the predicted distances between the drones. Finally, get the errors to
        # the desired distances.
        self.predicted_rel_positions = array_operations.multiarray2np(data)
        self.predicted_distances = np.linalg.norm(self.predicted_rel_positions,
                                                  axis=2)
        self.errors = error_functions.simple_differences(
                self.desired_distances, self.predicted_distances)

        # Apply sign to errors, so they are symmetric.
        sign_matrix = np.triu(np.ones((3)), 1) + np.tril(-1*np.ones((3)), -1)
        self.errors *= sign_matrix
        # P gain
        Kp = 1.2

        # Calculate and send the final control variable
        control_u = self.errors.sum(axis=1) * Kp

        self.control_var_pub.publish(array_operations.np2multiarray(control_u))
        return

    def pat_gen_start_callback(self, data):
        """
        Store the desired distances, and allow the controller to start
        """
        self.desired_distances = array_operations.multiarray2np_sqr(data.data)
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
