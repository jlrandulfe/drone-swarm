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
from pycopter.srv import PycopterStartPositions
from pycopter.srv import PycopterStartStop
from pycopter.srv import PycopterStartStopResponse


class DroneSwarmNode():

    def __init__(self):
        self.start = False
        self.stop = False
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
        self.init_formation()

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
        try:
            setup_pycopter = rospy.ServiceProxy("supervisor/pycopter",
                                                PycopterStartPositions)
            resp = setup_pycopter(True)
            self.n_drones = resp.n_rows
            positions = resp.data
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            return -1
        # Initial conditions
        for i in range(self.n_drones):
            if i == 0:
                xyz_0 = np.array([positions[0], positions[1], 0])
            else:
                new_pos = np.array([positions[2*i], positions[2*i+1], 0])
                xyz_0 = np.vstack((xyz_0, new_pos))
        att_0 = np.array([0.0, 0.0, 0.0])
        pqr_0 = np.array([0.0, 0.0, 0.0])
        v_ned_0 = np.array([0.0, 0.0, 0.0])
        w_0 = np.array([0.0, 0.0, 0.0, 0.0])
        for i in range(self.n_drones):
            self.drones.append(quad.quadrotor(1, self.m, self.l, self.J, 
                    self.CDl, self.CDr, self.kt, self.km, self.kw, att_0,
                    pqr_0, xyz_0[i], v_ned_0, w_0))
            if i <= self.n_drones/2:
                self.drones[i].yaw_d = (2*np.pi/self.n_drones) * i
            else:
                self.drones[i].yaw_d = -((2*np.pi/self.n_drones)
                                         * (self.n_drones-i))
        # Desired heading.
        #TODO: Generalize for n drones

        # self.drones[0].yaw_d = -np.pi
        # self.drones[1].yaw_d = np.pi/2
        # self.drones[2].yaw_d = 0
        # Instantiate the simulation class
        tf=60
        self.dt=5e-2
        self.time = np.linspace(0, tf, tf/self.dt)
        self.quad_sim = simulation.SimNQuads(self.drones, self.fc, self.time,
                                             self.n_drones)
        rospy.loginfo("The drone simulation has been set-up")
        return 0

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
        return

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

    def handle_start_stop(self, req):
        if req.stop:
            self.stop = True
            rospy.logwarn("Received STOP command")
        if req.start:
            self.start = True
            rospy.logwarn("Received START command")
        return PycopterStartStopResponse(True)

    def run(self):
        """
        Main execution routine of the DroneSwarm class

        Get initialization parameters from the supervisor node, and then
        waits until this node sends the start command.

        Afterwards, it goes to a loop where the PyCopter simulation is
        run. Exits when the simulation final time is reached.
        """
        # Wait until the supervisor server is ready
        rospy.loginfo("Waiting for supervisor service")
        rospy.wait_for_service("supervisor/pycopter")
        init = self.init_drones()
        # If there was an error, exit execution
        if init == -1:
            return
        rospy.loginfo("Setting start/stop service up")
        rospy.Service("pycopter/start_stop", PycopterStartStop,
                      self.handle_start_stop)
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
