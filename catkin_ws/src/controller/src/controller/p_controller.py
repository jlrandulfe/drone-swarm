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
from controller import array_operations
from controller import error_functions
from pycopter.srv import DroneSwarmMultiArray


def dist(vector):
    length = np.sqrt(pow(vector[0],2)+pow(vector[1],2))
    return length

class PControlNode():

    def __init__(self):
        self.start = False
        self.new_it = False
        self.n_drones = 0
        self.timestamp = 0
        self.movement = "static"
        self.velocity = np.array([0,0])         # [m/s, m/s]
        self.sin_amplitude = np.array([0, 0])   # [m, m]
        self.sin_frequency = 0.0                # [Hz]
        self.timestamp = 0                      # [ms]
        # Errors matrix
        self.errors = np.array([[0, 1, 2],[1, 0, 3],[2, 3, 0]])
        self.predicted_rel_positions = np.array([])
        self.predicted_distances = np.array([])
        self.desired_distances = np.array([])

        # Control variables matrix
        self.control_u = np.array([])

        # Kalman filter topic susbscribers
        rospy.Subscriber("kalman/pos_estimation",
                         std_msgs.msg.Float64MultiArray,
                         self.kalman_callback, queue_size=1)

        # Drone controller topic publisher
        self.control_var_pub = rospy.Publisher(
                "controller/control_value",
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
        self.timestamp = data.layout.data_offset
        # Get the relative positions from the Kalman node.
        self.predicted_rel_positions = array_operations.multiarray2np(data)
        self.new_it = True
        return

    def pat_gen_start(self):
        """
        Store the desired distances, and allow the controller to start
        """
        try:
            setup_pycopter = rospy.ServiceProxy("supervisor/kalman",
                                                DroneSwarmMultiArray)
            resp = setup_pycopter(True)
            self.n_drones = resp.n_rows
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            return -1
        self.desired_distances = array_operations.multiarray2np_sqr(resp)

        # self.desired_distances = array_operations.multiarray2np_sqr(data.data)
        self.start = True
        rospy.loginfo("Controller: Received formation from supervisor.")
        return 0

    def gradient_descent_control(self, Kp=1.2):
        """
        Apply gradient descent for finding the control action

        For a given N number of agents, calculate the control action so
        it minimizes the accumulated error of the drones positions.
        """
        # Calculate the predicted distances between the drones. Then, get the
        # errors to the desired distances.
        self.predicted_distances = np.linalg.norm(self.predicted_rel_positions,
                                                  axis=2)
        self.errors = error_functions.simple_differences(
                self.desired_distances, self.predicted_distances)

        # Apply sign to errors, so they are symmetric.
        sign_matrix = np.triu(np.ones((3)), 1) + np.tril(-1*np.ones((3)), -1)
        self.errors *= sign_matrix

        # Unit vectors calculation. Create a virtual axis so the division is
        # dimension meaningful.
        # print('predict dist',self.predicted_distances)
        # print('predict pos',self.predicted_rel_positions)
        # print('drone amount', len(self.predicted_distances))
        # unit_vectors = (self.predicted_rel_positions
        #                 / self.predicted_distances[:,:,None])
        num_drone = len(self.predicted_distances)
        unit_vectors = np.zeros((num_drone, num_drone, 2))
        for x in range(0,len(self.predicted_distances)):
            for y in range(0,len(self.predicted_distances)):
                if x == y:
                    unit_vectors[x][y][0] = 0.0
                    unit_vectors[x][y][1] = 0.0
                else:
                    unit_vectors[x][y][0] = self.predicted_rel_positions[x][y][0]/self.predicted_distances[x][y]
                    unit_vectors[x][y][1] = self.predicted_rel_positions[x][y][1]/self.predicted_distances[x][y]

        # print("unit vec", unit_vectors)
        # Calculate and send the final control variable
        self.control_u = (self.errors[:,:,None] * unit_vectors).sum(axis=1) * Kp
        return

    def set_leader_velocity(self):
        if self.movement == "static":
            pass
        elif self.movement == "linear":
            self.control_u[0] += self.velocity
        elif self.movement == "sinusoidal":
            self.control_u[0] += self.sin_amplitude * np.sin(self.sin_frequency
                                                             * self.timestamp)
        else:
            raise ValueError("Unrecognized movement type")
        return

    def run(self):
        """
        Main routine. Wait for initialization and then do the main loop

        It waits until the supervisor node service is active. Then, it
        requests the desired formation from it.

        Afterwards, it enters the main loop, where it continuously
        waits for position predictions. After receiving a prediction, it
        calculates the control action by using a gradient-descent based
        controller. Finally, it applies to the leader of the formation
        the desired movement of the swarm on top of the control action.
        """
        rospy.loginfo("Controller started. Waiting for a desired formation")
        rospy.wait_for_service("/supervisor/kalman")
        rospy.loginfo("Online")
        # Connect to the supervisor service and get the desired formation.
        self.pat_gen_start()

        # Main loop. Wait for predictions and calculate the control action
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.start and self.new_it:
                self.gradient_descent_control()
                self.set_leader_velocity()
                self.control_var_pub.publish(array_operations.np2multiarray(
                        self.control_u))
                rospy.loginfo("Controller: published U {}, {}, {}".format(
                        self.control_u[0], self.control_u[1], self.control_u[2]))
                self.new_it = False
            rate.sleep()
        rospy.spin()
        return


def main():
    # Instantiate the error_estimator node class and run it
    controller = PControlNode()
    controller.run()
    return
