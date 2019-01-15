import numpy as np
from scipy import linalg as la
from matplotlib import pyplot as pl
from matplotlib import mlab as mlab

class Kalman:

	def __init__(self):

		# Kalman matrices init!

		# Observation matrix H 1/d * [ px, py ] JACOBIAN because we've non linear measure
		self.H = np.array([[1, 1]])
		# time step
		self.dt = 5e-2
		# distance sensor noise
		self.radar_sigma = 0.1
		# measurement covariance matrix
		self.R = np.array([[0.01],
						[0.01]])

		# States p01_x and p01_y
		self.state_X = np.array([0, 0])

		# P Covariance matrix for the position meas
		self.cov_P = np.array([[0.5, 0],
					[0, 0.5]])

		# predicted state
		self.predict_state_X = np.array([[0, 0]]).T

		self.predict_cov_P = np.array([[0, 0],
					[0, 0]])
		
		self.F = np.array([[1, 0],
					[0, 1]])
		# Matrix G is for determine the dynamics of the states based on the sensor measures in this case velocity to
		# estimate position.!
		self.G = np.array([[self.dt, 0],
					[0, self.dt]])
		# variance vx and vy
		#vel_sigma = np.array([[0.01, 0],
							#[0, 0.01]])
		vel_sigma = 0.1
		#self.Q = np.outer(self.G*vel_sigma, vel_sigma*self.G.transpose())
		self.Q = (self.G * vel_sigma).dot(vel_sigma * self.G.transpose())
		self.K = np.array([[0.1, 0.1]]).T

		#self.i = 0
		self.err_class = 0

		#pl.ion()
		#self.fig, self.axis = pl.subplots(3, 1)


	def predict(self, state_sim, relative_velocities):
		print(" PREDICTION ")
		print("\n")
		# predict the position in the plane of the drone
		# How the position evolvers -> dynamics
		#self.next_pos_x = self.state_X[0] + relative_velocities[0] * self.dt
		#self.next_pos_y = self.state_X[1] + relative_velocities[1] * self.dt

		# gaussian distributions for position in x and y they are stochastic variables. That is why we estimate position
		# and we calculate the variance of that estimation. mean + var : normal distr


		#print(" F size: ", self.F.size)
		#print("\n")

		#print(" state_X size: ", self.state_X.size)
		#print("\n")

		#print(" G size: ", self.G.shape)
		#print("\n")

		#print(" rel_velocities size: ", relative_velocities.shape)
		#print("\n")

		#print(" first mult size: ", self.F.dot(self.state_X).shape)
		#print("\n")

		#print(" second mult : ", self.G.dot(relative_velocities).reshape(2, 1))
		#print("\n")

		#print(" Q size: ", self.Q.size)
		#print("\n")
		self.state_X = state_sim

		self.predict_state_X = self.F.dot(self.state_X) + self.G.dot(relative_velocities.reshape(2, 1))
		self.predict_cov_P = self.F.dot(self.cov_P).dot(self.F.transpose()) + self.Q
		self.state_X = self.predict_state_X
		self.cov_P = self.predict_cov_P
		print(" predict_State : ", self.predict_state_X)
		print("\n")

		print(" predict_cov_P : ", self.predict_cov_P)
		print("\n")

	def distance (self, x, y):

		# norm distance calculation because we have a radio frequency sensor that measures the distance between drones
		sum_sq = (x * x) + (y * y)
		d = np.sqrt(sum_sq)

		return d

	def update(self, state_sim, distance_sensor):
		print(" UPDATE ") # STEP K
		print("\n")
		# correct the KF
		#print(" predict_State size: ", self.predict_state_X.size)
		#print("\n")
		#previous_state = self.predict_state_X
		#previous_cov = self.predict_cov_P

		# due the fact that the distance has a non linear relation with the position we calculate the jacobian
		# scalar * 2D array
		self.state_X = state_sim
		dist = (self.distance(self.state_X[0], self.state_X[1]))
		#print(" dist: ", dist)
		#print("\n")

		if dist.any() == 0:
			pass
		else:
			self.H = (1 / dist) * self.state_X.transpose()

			#print(" H ", self.H)
		#print("\n")
		#print(" Previous state: ", previous_state)
		#print("\n")

		#print(" H shape: ", self.H.shape)
		#print("\n")

		#print(" H : ", self.H)
		#print("\n")

		#print(" previous_State size: ", previous_state.shape)
		#print("\n")

		#print(" first mult size: ", (self.H * self.radar_sigma).shape)
		#print("\n")

		#print(" second mult size: ", (self.radar_sigma * self.H.reshape(2,1)).shape)
		#print("\n")

		#print(" H_trans: ", self.H.transpose().shape)
		#print("\n")

		self.R = (self.H * self.radar_sigma).dot((self.radar_sigma * self.H.reshape(2, 1)))

		#print(" R size: ", self.R.shape)
		#print("\n")

		#print(" R: ", self.R)
		#print("\n")

		S = self.H.dot(self.cov_P).dot(self.H.reshape(2, 1)) + self.R

		#print(" S size: ", S.shape)
		#print("\n")
		#print(" P : ", self.cov_P)
		#print("\n")

		#print(" S : ", S)
		#print("\n")

		#print(" first mult size: ", (self.H * self.radar_sigma))
		#print("\n")

		if np.size(S) == 1: ## WHATS WRONG HERE?
			self.K = self.cov_P.dot(self.H.reshape(2, 1)) / S

			#print(" P * Ht size: ", (self.cov_P.dot(self.H.reshape(2, 1))).shape)
			#print("\n")

			#print(" P * Ht : ", (self.cov_P.dot(self.H.reshape(2, 1))))
			#print("\n")

			#print(" K size: ", self.K.shape)
			#print("\n")

			#print(" K : ", self.K)
			#print("\n")
			#print(" prueba2: ", self.H.dot(self.cov_P))
			#print("\n")
			#print(" prueba1: ", np.outer(self.K, self.H.dot(self.cov_P)))
			#print("\n")
			self.cov_P = self.cov_P - np.outer(self.K, self.H.dot(self.cov_P))
		else:
			self.K = self.cov_P.dot(self.H.reshape(2, 1)).dot(la.inv(S))
			self.cov_P = self.cov_P - np.outer(self.K, self.H.dot(self.cov_P))

		self.state_X = self.state_X + self.K * (self.error(distance_sensor))

		#print(" H : ", self.H)
		#print("\n")
		#print(" cov_P: ", self.cov_P)
		#print("\n")

		print(" state_X: ", self.state_X)
		print("\n")

		print(" cov_P: ", self.cov_P)
		print("\n")

	def error(self, distance_sensor):

		# for tracking the error: here we compare the distance that we obtain with the sensor vs the distance calculated
		# by the position estimation.
		dist_estimation = self.distance(self.state_X[0], self.state_X[1])
		err = distance_sensor - dist_estimation
		self.err_class = float(err)
		#self.err_plot[self.i] = distance_sensor - dist_estimation
		rmse = np.sqrt(dist_estimation*dist_estimation - distance_sensor*distance_sensor)
		#self.i = self.i +1
		#print(" Error: ", err)
		#print("\n")

		#print(" RMSE CALC: ", rmse)
		#print("\n")
		return err

	#def variance_calculation :

	def animation(self, it, tf, time, err_plt):

		err_plt[it] = self.err_class

		self.fig.tight_layout()

		xpl = 10
		ypl = 1
		
		y_y_lim =10
		xlimits0 = np.linspace(-2.5, xpl, 300)# Limits for pos in axis 'x' and 'y'
		xlimits1 = np.linspace(-2.5, y_y_lim, 300)
	
		# Plot of X position relative
		self.axis[0].clear()
		self.axis[0].grid("on")
		pxgauss = mlab.normpdf(xlimits0, self.state_X[0], np.sqrt(self.cov_P[0, 0]))
		self.axis[0].plot(xlimits0, pxgauss)
		self.axis[0].fill_between(xlimits0, pxgauss, color='cyan')
		self.axis[0].set_xlim([-xpl, xpl])
		self.axis[0].set_ylim([0, ypl])
		self.axis[0].set_yticks([0, 0.5 * ypl, ypl])
		self.axis[0].set_title("Estimated relative X position")
		self.axis[0].set_xlabel("[m]")
		self.axis[0].arrow(0, 0, 0, ypl, \
					  head_width=0.05, head_length=0.1, fc='k', ec='k')

		# Plot of Y position relative
		self.axis[1].clear()
		self.axis[1].grid("on")
		pygauss = mlab.normpdf(xlimits1, self.state_X[1], np.sqrt(self.cov_P[1, 1]))
		self.axis[1].plot(xlimits1, pygauss)
		self.axis[1].fill_between(xlimits1, pygauss, color='cyan')
		self.axis[1].set_xlim([-xpl, xpl])
		self.axis[1].set_ylim([0, ypl])
		self.axis[1].set_yticks([0, 0.5 * ypl, ypl])
		self.axis[1].set_title("Estimated relative Y velocity")
		self.axis[1].set_xlabel("[m]")
		self.axis[1].arrow(0, 0, 0, ypl, \
					  head_width=0.05, head_length=0.1, fc='k', ec='k')

		# Plot of distance error
		self.axis[2].clear()
		self.axis[2].grid("on")
		self.axis[2].plot(time[0:it], err_plt[0:it], 'r')
		self.axis[2].set_xlim([0, tf])
		self.axis[2].set_ylim([0, 1])
		self.axis[2].set_title("Error in distance estimation")
		self.axis[2].set_xlabel("[m]")
	