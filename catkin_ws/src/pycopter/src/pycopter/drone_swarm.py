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
from pycopter import formation_distance as form
from pycopter import simulation
import pycopter.quadrotor as quad


class DroneSwarmNode():

    def __init__(self):
        self.start = False
        self.drones = []
        self.n_drones = 0
        self.fc = None
        self.U = None

        # Quadrotors physical properties
        self.m = 0.65 # Kg
        self.l = 0.23 # m
        self.Jxx = 7.5e-3 # Kg/m^2
        self.Jyy = self.Jxx
        self.Jzz = 1.3e-2
        self.Jxy = 0
        self.Jxz = 0
        self.Jyz = 0
        self.J = np.array([[self.Jxx, self.Jxy, self.Jxz],
                           [self.Jxy, self.Jyy, self.Jyz],
                           [self.Jxz, self.Jyz, self.Jzz]])
        self.CDl = 9e-3
        self.CDr = 9e-4
        self.kt = 3.13e-5  # Ns^2
        self.km = 7.5e-7   # Ns^2
        self.kw = 1/0.18   # rad/s

        # Initialize the quadrotors
        self.init_drones()
        self.init_formation()

        # Desired heading.
        #TODO: Generalize for n drones
        self.drones[0].yaw_d = -np.pi
        self.drones[1].yaw_d = np.pi/2
        self.drones[2].yaw_d = 0

        # Instantiate the sim class
        tf=60
        self.dt=5e-2
        self.time = np.linspace(0, tf, tf/self.dt)
        self.quad_sim = simulation.SimNQuads(self.drones, self.fc, self.time)

        # ROS subscriber
        rospy.Subscriber("controller/control_value",
                         std_msgs.msg.Float64MultiArray,
                         self.controller_callback, queue_size=1)

        # ROS publishers
        self.positions_pub = rospy.Publisher(
                "pycopter/positions",
                std_msgs.msg.Float64MultiArray,
                queue_size=10)
        self.velocities_pub = rospy.Publisher(
                "pycopter/velocities",
                std_msgs.msg.Float64MultiArray,
                queue_size=10)

    def init_drones(self):
        """
        Initialize the quadrotor drones and store them in a list
        """
        self.start = True
        self.n_drones = 3
        # Initial conditions
        att_0 = np.array([0.0, 0.0, 0.0])
        pqr_0 = np.array([0.0, 0.0, 0.0])
        #TODO: Generalize for n drones
        xyz_0 = [np.array([1.0, 1.2, 0.0]),
                 np.array([1.2, 2.0, 0.0]),
                 np.array([-1.1, 2.6, 0.0])]
        v_ned_0 = np.array([0.0, 0.0, 0.0])
        w_0 = np.array([0.0, 0.0, 0.0, 0.0])
        for i in range(self.n_drones):
            self.drones.append(quad.quadrotor(1, self.m, self.l, self.J, 
                    self.CDl, self.CDr, self.kt, self.km, self.kw, att_0,
                    pqr_0, xyz_0[i], v_ned_0, w_0))

    def init_formation(self):
        """
        Set the formation control
        """
        # Formation Control
        # Shape
        side = 8
        Btriang = np.array([[1, 0, -1],[-1, 1, 0],[0, -1, 1]])
        dtriang = np.array([side, side, side])

        # Motion
        mu = 0e-2*np.array([1, 1, 1])
        tilde_mu = 0e-2*np.array([1, 1, 1])

        self.fc = form.formation_distance(2, 1, dtriang, mu, tilde_mu, Btriang,
                                          5e-2, 5e-1)

    def np2multiarray(self, data, n_coords=2):
        # Define the 2 dimensions of the array.
        dim_1 = std_msgs.msg.MultiArrayDimension(label="drone_n",
                                                 size=self.n_drones,
                                                 stride=self.n_drones*n_coords)
        dim_2 = std_msgs.msg.MultiArrayDimension(label="coords", size=n_coords,
                                                 stride=n_coords)
        # Create the layout of the message, necessary for deserializing it.
        layout = std_msgs.msg.MultiArrayLayout()
        layout.dim.append(dim_1)
        layout.dim.append(dim_2)
        # Create the output message with the data and the created layout.
        message = std_msgs.msg.Float64MultiArray()
        message.layout = layout
        message.data = data.tolist()
        return message

    def controller_callback(self, data):
        self.U = data.data
        return

    def run(self):
        while not self.start:
            pass
        it = 0
        rate = rospy.Rate(100)
        while it < len(self.time):
            t = self.time[it]
            it += 1
            output = self.quad_sim.new_iteration(t, self.dt, self.U)
            if (output == -1):
                rospy.logerr("Pycopter simulator crashed")
                break
            else:
                X, V = output
                positions_message = self.np2multiarray(X)
                velocities_message = self.np2multiarray(V)
                self.positions_pub.publish(positions_message)
                self.velocities_pub.publish(velocities_message)
            rate.sleep()
        self.quad_sim.final_plots(self.time)
        return


def main():
    # Instantiate the error_estimator node class and run it
    drone_swarm = DroneSwarmNode()
    drone_swarm.run()
    return
