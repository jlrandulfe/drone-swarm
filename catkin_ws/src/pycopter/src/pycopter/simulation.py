#!/usr/bin/env python3
# Standard libraries
# Third-party libraries
import matplotlib.pyplot as plt
import numpy as np
import rospy
# Local libraries
from pycopter import quadlog
from pycopter import animation as ani


class SimNQuads():

    def __init__(self, quads, fc, time, ndrones=3, alt_d=0.2, frames=50):

        self.test = False
        self.ndrones = ndrones
        # Extract quadcopters from list
        self.quads = quads

        self.fc = fc

        # Simulation parameters
        self.it = 0

        # Data log
        self.qlogs = []
        for _ in range(ndrones):
            self.qlogs.append(quadlog.quadlog(time))
        self.Ed_log = np.zeros((time.size, 4))

        # Plots
        self.quadcolor = ["r", "g", "b", "c", "m", "y", "k", "r", "b"]
        plt.close("all")
        plt.ion()
        self.fig = plt.figure(0)
        self.axis3d = self.fig.add_subplot(111, projection='3d')

        self.init_area = 5
        self.s = 2
        self.alt_d = alt_d
        
        self.counter_reach_alt = 0


        self.frames = frames

    def get_errors(self, errors):
        errors_array = np.array(errors[:-1])
        errors_array = errors_array.reshape([self.ndrones,  self.ndrones])
        errors_list = [errors_array[0,1], errors_array[0,2], errors_array[1,2]]
        # Append the system error at the end of the list
        errors_list.append(errors[-1])
        return errors_list

    def new_iteration(self, t, dt, U=None, errors=None):
        # Simulation
        X = np.array([])
        V = np.array([])
        for i in range(self.ndrones):
            X = np.append(X, self.quads[i].xyz[0:2])
            V = np.append(V, self.quads[i].v_ned[0:2])
        if t<5:
            U = []
            for i in range(self.ndrones):
                U.append(0)
                U.append(0)
            print('No U Present')


        for i in range(self.ndrones):
            # print("z: ", self.quads[i].xyz[2], "des_z: ", self.alt_d)
            if self.quads[i].xyz[2] < -self.alt_d and self.counter_reach_alt < self.ndrones*5:
                # print("Reached Altitude")
                self.counter_reach_alt+=1

            if self.test:
                self.quads[i].set_a_2D_alt_lya(U[2*i:2*i+2], -self.alt_d)
            else:
                self.quads[i].set_v_2D_alt_lya(U[2*i:2*i+2], -self.alt_d)
                rospy.logwarn('drone {} velocity: {} {}'.format(i, U[2*i], U[2*i+1]))
            self.quads[i].step(dt)

        # Animation
        if self.it%self.frames == 0:
            plt.figure(0)
            self.axis3d.cla()
            for i in range(self.ndrones):
                ani.draw3d(self.axis3d, self.quads[i].xyz,
                           self.quads[i].Rot_bn(), self.quadcolor[i])
            self.axis3d.set_xlim(-5, 5)
            self.axis3d.set_ylim(-5, 5)
            self.axis3d.set_zlim(0, 10)
            self.axis3d.set_xlabel('South [m]')
            self.axis3d.set_ylabel('East [m]')
            self.axis3d.set_zlabel('Up [m]')
            self.axis3d.set_title("Time %.3f s" %t)
            plt.pause(0.001)
            plt.draw()
            
            plt.figure(1)
            plt.clf()
            ani.draw2d(1, X, self.fc, self.quadcolor, self.ndrones)
            if self.ndrones == 3:
                ani.draw_edges(1, X, self.fc, -1)
            plt.xlabel('South [m]')
            plt.ylabel('West [m]')
            plt.title('2D Map')
            plt.xlim(-self.s*self.init_area, self.s*self.init_area)
            plt.ylim(-self.s*self.init_area, self.s*self.init_area)
            plt.grid()
            plt.pause(0.001)
            plt.draw()

        # Log
        for i in range(self.ndrones):
            self.qlogs[i].xyz_h[self.it, :] = self.quads[i].xyz
            self.qlogs[i].att_h[self.it, :] = self.quads[i].att
            self.qlogs[i].w_h[self.it, :] = self.quads[i].w
            self.qlogs[i].v_ned_h[self.it, :] = self.quads[i].v_ned
        if errors:
            self.Ed_log[self.it,:] = self.get_errors(errors)
        else:
            self.Ed_log[self.it, :] = [0] * 4

        self.it+=1
        
        # Stop if crash
        if (self.quads[0].crashed==1 or self.quads[1].crashed==1 or self.quads[2].crashed==1):
            return -1
        return (X, V)

    def final_plots(self, time, it):

        plt.figure(1)
        plt.title("2D Position [m]")
        for i in range(self.ndrones):
            plt.plot(self.qlogs[i].xyz_h[:, 0], self.qlogs[i].xyz_h[:, 1],
                     label="q{}".format(i+1), color=self.quadcolor[i])
        plt.xlabel("East")
        plt.ylabel("South")
        plt.legend()

        plt.figure(2)
        for i in range(self.ndrones):
            plt.plot(time[0:it], self.qlogs[i].att_h[:, 2][0:it], label="yaw q{}".format(i+1))
        plt.xlabel("Time [s]")
        plt.ylabel("Yaw [rad]")
        plt.grid()
        plt.legend()

        plt.figure(3)
        for i in range(self.ndrones):
            plt.plot(time[0:it], -self.qlogs[i].xyz_h[:, 2][0:it],
                     label="$q_{}$".format(i+1))
        plt.xlabel("Time [s]")
        plt.ylabel("Altitude [m]")
        plt.grid()
        plt.legend(loc=2)

        plt.figure(4)
        plt.plot(time[0:it], self.Ed_log[:, 0][0:it], label="$e_{12}$")
        plt.plot(time[0:it], self.Ed_log[:, 1][0:it], label="$e_{13}$")
        plt.plot(time[0:it], self.Ed_log[:, 2][0:it], label="$e_{23}$")
        plt.plot(time[0:it], self.Ed_log[:, 3][0:it], label="$e_{T}$")
        plt.xlabel("Time [s]")
        plt.ylabel("Formation distance error [m]")
        plt.grid()
        plt.legend()

        try:
            plt.pause(0)
        except:
            pass
        return
