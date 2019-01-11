#!/usr/bin/env python3
# Standard libraries
# Third-party libraries
import matplotlib.pyplot as plt
import numpy as np
# Local libraries
from pycopter import quadlog
from pycopter import animation as ani


class Sim3Quads():

    def __init__(self, quads, fc, time, alt_d=4, frames=100):
        # Extract quadcopters from list
        self.q1 = quads[0]
        self.q2 = quads[1]
        self.q3 = quads[2]

        self.fc = fc

        # Simulation parameters
        # time = np.linspace(0, tf, tf/dt)
        self.it = 0

        # Data log
        self.q1_log = quadlog.quadlog(time)
        self.q2_log = quadlog.quadlog(time)
        self.q3_log = quadlog.quadlog(time)
        self.Ed_log = np.zeros((time.size, self.fc.edges))

        # Plots
        self.quadcolor = ['r', 'g', 'b']
        plt.close("all")
        plt.ion()
        self.fig = plt.figure(0)
        self.axis3d = self.fig.add_subplot(111, projection='3d')

        self.init_area = 5
        self.s = 2
        self.alt_d = alt_d

        self.frames = frames

    def new_iteration(self, t, dt):
        # Simulation
        X = np.append(self.q1.xyz[0:2], np.append(self.q2.xyz[0:2],
                      self.q3.xyz[0:2]))
        V = np.append(self.q1.v_ned[0:2], np.append(self.q2.v_ned[0:2],
                      self.q3.v_ned[0:2]))
        U = self.fc.u_acc(X, V)

        self.q1.set_a_2D_alt_lya(U[0:2], -self.alt_d)
        self.q2.set_a_2D_alt_lya(U[2:4], -self.alt_d)
        self.q3.set_a_2D_alt_lya(U[4:6], -self.alt_d)

        self.q1.step(dt)
        self.q2.step(dt)
        self.q3.step(dt)

        # Animation
        if self.it%self.frames == 0:

            plt.figure(0)
            self.axis3d.cla()
            ani.draw3d(self.axis3d, self.q1.xyz, self.q1.Rot_bn(),
                       self.quadcolor[0])
            ani.draw3d(self.axis3d, self.q2.xyz, self.q2.Rot_bn(),
                       self.quadcolor[1])
            ani.draw3d(self.axis3d, self.q3.xyz, self.q3.Rot_bn(),
                       self.quadcolor[2])
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
            ani.draw2d(1, X, self.fc, self.quadcolor)
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
        self.q1_log.xyz_h[self.it, :] = self.q1.xyz
        self.q1_log.att_h[self.it, :] = self.q1.att
        self.q1_log.w_h[self.it, :] = self.q1.w
        self.q1_log.v_ned_h[self.it, :] = self.q1.v_ned

        self.q2_log.xyz_h[self.it, :] = self.q2.xyz
        self.q2_log.att_h[self.it, :] = self.q2.att
        self.q2_log.w_h[self.it, :] = self.q2.w
        self.q2_log.v_ned_h[self.it, :] = self.q2.v_ned

        self.q3_log.xyz_h[self.it, :] = self.q3.xyz
        self.q3_log.att_h[self.it, :] = self.q3.att
        self.q3_log.w_h[self.it, :] = self.q3.w
        self.q3_log.v_ned_h[self.it, :] = self.q3.v_ned

        self.Ed_log[self.it, :] = self.fc.Ed

        self.it+=1
        
        # Stop if crash
        if (self.q1.crashed==1 or self.q2.crashed==1 or self.q3.crashed==1):
            return -1
        return 0

    def final_plots(self, time):

        plt.figure(1)
        plt.title("2D Position [m]")
        plt.plot(self.q1_log.xyz_h[:, 0], self.q1_log.xyz_h[:, 1], label="q1", color=self.quadcolor[0])
        plt.plot(self.q2_log.xyz_h[:, 0], self.q2_log.xyz_h[:, 1], label="q2", color=self.quadcolor[1])
        plt.plot(self.q3_log.xyz_h[:, 0], self.q3_log.xyz_h[:, 1], label="q3", color=self.quadcolor[2])
        plt.xlabel("East")
        plt.ylabel("South")
        plt.legend()

        plt.figure(2)
        plt.plot(time, self.q1_log.att_h[:, 2], label="yaw q1")
        plt.plot(time, self.q2_log.att_h[:, 2], label="yaw q2")
        plt.plot(time, self.q3_log.att_h[:, 2], label="yaw q3")
        plt.xlabel("Time [s]")
        plt.ylabel("Yaw [rad]")
        plt.grid()
        plt.legend()

        plt.figure(3)
        plt.plot(time, -self.q1_log.xyz_h[:, 2], label="$q_1$")
        plt.plot(time, -self.q2_log.xyz_h[:, 2], label="$q_2$")
        plt.plot(time, -self.q3_log.xyz_h[:, 2], label="$q_3$")
        plt.xlabel("Time [s]")
        plt.ylabel("Altitude [m]")
        plt.grid()
        plt.legend(loc=2)

        plt.figure(4)
        plt.plot(time, self.Ed_log[:, 0], label="$e_1$")
        plt.plot(time, self.Ed_log[:, 1], label="$e_2$")
        plt.plot(time, self.Ed_log[:, 2], label="$e_3$")
        plt.xlabel("Time [s]")
        plt.ylabel("Formation distance error [m]")
        plt.grid()
        plt.legend()

        try:
            plt.pause(0)
        except:
            pass
        return
