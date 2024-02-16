import matplotlib.pyplot as plt

class Visualizer:

    def __init__(self, state):
                
        # Create a 3D axis
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Drone 3D Trajectory')

        # Set limits for your plot (change these as per your needs)
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([0, 20])

        # Create a line object for trajectory
        (self.line,) = self.ax.plot([state['x']], [state['y']], [state['z']], lw=2, c='r')

    def update_plot(self, state):
        """Update the trajectory plot based on drone's position."""
        # Append new position
        x_data, y_data, z_data = self.line.get_data_3d()
        x_data.append(state['x'])
        y_data.append(state['y'])
        z_data.append(state['z'])

        # Set updated data
        self.line.set_data([x_data, y_data])
        self.line.set_3d_properties(z_data)

        # Draw updated plot
        plt.draw()
        plt.pause(0.001)  # Adjust for the desired refresh rate
