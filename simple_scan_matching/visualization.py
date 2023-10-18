import numpy as np
import matplotlib.pyplot as plt

from filterpy.gh import GHFilter

class Visualization:

	def __init__(self, width, height, init_robot_x, init_robot_y, dt = 1.0, g=0.2, h=0.1):

		
		# Create environment with obstacles
		self.env = np.zeros((height, width))
		
		self.half_width, self.half_height = self.env.shape[1] // 2, self.env.shape[0] // 2
		self.extent = [-self.half_width, self.half_width, -self.half_height, self.half_height]
		if width >= 20:
			self.ticks = width // 20
		else:
			self.ticks = 1
		self.fig, self.ax = plt.subplots()

		# The positions to plot the trajectory
		self.positions_x = [init_robot_x]
		self.positions_y = [init_robot_y]

		# The positions to plot the filtered trajectory
		self.filtered_positions_x = [init_robot_x]
		self.filtered_positions_y = [init_robot_y]

		self.sc = self.ax.scatter(self.positions_x, self.positions_y , c= 'b', s = 10)

		# The filter section
		x_0  = np.array([init_robot_x, init_robot_y])
		dx_0 = np.array([0.0, 0.0])

		self.dt = dt
		self.filter = GHFilter(x=x_0, dx=dx_0, dt=self.dt, g=g, h=h)

		plt.ion()
		plt.show()
		
		
	
	def add_grid_and_labels(self):
		# Add grid
		self.ax.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
		self.ax.set_xticks(np.arange(-self.half_width, self.half_width + 1, self.ticks))
		self.ax.set_yticks(np.arange(-self.half_height, self.half_height + 1, self.ticks))
		
		# Add axis arrows
		self.ax.arrow(0, -self.half_height, 0, 2 * self.half_height - 1, head_width=2, head_length=2, fc='k', ec='k')
		self.ax.arrow(-self.half_width, 0, 2 * self.half_width - 1, 0, head_width=2, head_length=2, fc='k', ec='k')

		# Add axis labels
		self.ax.text(self.half_width - 5, 2, 'X', fontsize=12)
		self.ax.text(2, self.half_height - 5, 'Y', fontsize=12)

	def update_viz(self, robot_x, robot_y, robot_theta, robot_x2, robot_y2, robot_theta2):

		# update filter
		self.filter.update(z=np.array((robot_x, robot_y)))
		#print(' x =', self.filter.x)
		#print('dx =', self.filter.dx)
		#print("---------")

		# Update trajectories
		self.positions_x.append(robot_x)
		self.positions_y.append(robot_y)
		self.filtered_positions_x.append(robot_x2)
		self.filtered_positions_y.append(robot_y2)

		self.ax.clear()

		self.ax.imshow(self.env, cmap='binary', origin='lower', extent=self.extent)

		offset = 0.5

		# Calculate the new bounds based on the robot's position
		self.half_width = max(self.half_width, abs(robot_x) + 10)
		self.half_height = max(self.half_height, abs(robot_y) + 10)

		# Plot the robot position and heading
		self.ax.plot(robot_x, robot_y, 'ro', markersize=5)
		self.ax.plot([robot_x, robot_x + np.cos(robot_theta)], [robot_y, robot_y + np.sin(robot_theta)], 'b-', markersize=10)
		
        #self.ax.plot(robot_x2, robot_y2, 'ro', markersize=5)
		self.ax.plot([robot_x2, robot_x2 + np.cos(robot_theta2)], [robot_y2, robot_y2 + np.sin(robot_theta2)], 'g-', markersize=10)

		
		# Update the grid and labels
		self.add_grid_and_labels()

		# Plot the trajectories
		self.sc = self.ax.scatter(self.positions_x, self.positions_y , c= 'b', s = 20)
		self.sc = self.ax.scatter(self.filtered_positions_x, self.filtered_positions_y , c= 'g', s = 20)


		self.ax.set_aspect('equal')
		self.fig.canvas.draw_idle()

		plt.pause(0.00001)
