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
from pycopter import plotter
import pycopter.quadrotor as quad


class DroneSwarmNode():

    def __init__(self, n_drones=3):
        self.start = False
        self.drones = []
        self.n_drones = n_drones
        self.fc = None

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

        # Desired heading
        self.drones[0].yaw_d = -np.pi
        self.drones[1].yaw_d =  np.pi/2
        self.drones[2].yaw_d =  0

        # Instantiate the sim class
        tf=60
        self.dt=5e-2
        self.time = np.linspace(0, tf, tf/self.dt)
        self.quad_sim = plotter.Sim3Quads(self.drones, self.fc, self.time)

    def init_drones(self):
        """
        Initialize the quadrotor drones and store them in a list
        """
        # Initial conditions
        att_0 = np.array([0.0, 0.0, 0.0])
        pqr_0 = np.array([0.0, 0.0, 0.0])
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

    def run(self):
        # plotter.sim_and_plot_3quads(self.drones, self.fc)
        for t in self.time:
            self.quad_sim.new_iteration(t, self.dt)
        self.quad_sim.final_plots(self.time)
        return


def main():
    # Instantiate the error_estimator node class and run it
    drone_swarm = DroneSwarmNode()
    drone_swarm.run()
    return
