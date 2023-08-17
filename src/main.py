import numpy as np
import matplotlib.pyplot as plt

class Actuator:
    def __init__(self, position):
        self.thrust = 0.0
        self.position = position

    def set_thrust(self, thrust):
        self.thrust = thrust

    def get_thrust(self):
        return self.thrust

class DroneSimulator:
    def __init__(self):
        # Constants
        self.g = 9.81  # gravitational acceleration
        self.mass = 10.0  # mass of the drone in kg
        self.dt = 0.01  # time step
        self.drag_coeff = 0.01
        
        # Initial State
        self.position = np.array([0.0, 0.0, 30.0])
        self.velocity = np.array([0.0, 0.0, 0.0])

        # Actuators
        self.actuators = [
            Actuator(np.array([-1, 1, 0])),  # Top-left
            Actuator(np.array([1, 1, 0])),   # Top-right
            Actuator(np.array([-1, -1, 0])), # Bottom-left
            Actuator(np.array([1, -1, 0]))   # Bottom-right
        ]

    def compute_forces(self):
        gravity_force = np.array([0, 0, -self.mass * self.g])
        drag_force = -self.drag_coeff * self.velocity
        thrust_force = np.array([0, 0, np.sum([act.get_thrust() for act in self.actuators])])
        return gravity_force + drag_force + thrust_force

    def update(self):
        net_force = self.compute_forces()
        acceleration = net_force / self.mass
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt

        # Check for ground collision
        if self.position[2] < 0:
            self.position[2] = 0
            self.velocity[2] = 0

class Drone3DMatplotlibVisualizer:
    def __init__(self, drone_sim):
        self.drone_sim = drone_sim
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Setting the axes properties
        self.ax.set_xlim3d([-50.0, 50.0])
        self.ax.set_ylim3d([-50.0, 50.0])
        self.ax.set_zlim3d([0.0, 50.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Drone 3D Trajectory Visualization')

        # Initialize drone and actuator plots
        self.drone_dot, = self.ax.plot([], [], [], 'ro', markersize=10)
        self.actuator_dots = [self.ax.plot([], [], [], 'bo', markersize=6)[0] for _ in range(4)]

    def update_plot(self):
        # Update drone position
        x, y, z = self.drone_sim.position
        self.drone_dot.set_data(x, y)
        self.drone_dot.set_3d_properties(z)
        
        # Update actuator positions
        for actuator, dot in zip(self.drone_sim.actuators, self.actuator_dots):
            x, y, z = self.drone_sim.position + actuator.position
            dot.set_data(x, y)
            dot.set_3d_properties(z)

        plt.draw()
        plt.pause(0.005)

    def key_press_callback(self, event):
        thrust_amount = 500

        if event.key in ['1', '2', '3', '4']:
            idx = int(event.key) - 1
            self.drone_sim.actuators[idx].set_thrust(thrust_amount)
        elif event.key == 'w':
            for act in self.drone_sim.actuators:
                act.set_thrust(thrust_amount)
        elif event.key == 's':
            for act in self.drone_sim.actuators:
                act.set_thrust(-thrust_amount)
        else:
            for act in self.drone_sim.actuators:
                act.set_thrust(0)

    def key_release_callback(self, event):
        if event.key in ['1', '2', '3', '4']:
            idx = int(event.key) - 1
            self.drone_sim.actuators[idx].set_thrust(0)

    def run(self):
        # Bind the key press event
        self.fig.canvas.mpl_connect('key_press_event', self.key_press_callback)
        # Bind the key release event
        self.fig.canvas.mpl_connect('key_release_event', self.key_release_callback)
        
        
        while True:
            self.drone_sim.update()
            self.update_plot()
            if not plt.get_fignums():
                break

if __name__ == '__main__':
    drone_sim = DroneSimulator()
    visualizer = Drone3DMatplotlibVisualizer(drone_sim)
    visualizer.run()
