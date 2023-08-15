import numpy as np
import pygame
import matplotlib.pyplot as plt

class Actuator:
    def __init__(self):
        self.thrust = 0.0

    def set_thrust(self, thrust):
        self.thrust = thrust

    def get_thrust(self):
        return self.thrust

class DroneSimulator:
    def __init__(self):
        # Constants
        self.g = 9.81  # gravitational acceleration
        self.mass = 1.0  # mass of the drone in kg
        self.dt = 0.01  # time step
        self.drag_coeff = 0.01
        
        # Initial State
        self.position = np.array([0.0, 0.0, 30.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])

        # Manual inputs
        self.thrust = 0  # from propellers
        self.torque = np.array([0, 0, 0])  # roll, pitch, yaw torques

    def compute_forces(self):
        gravity_force = np.array([0, 0, -self.mass * self.g])
        drag_force = -self.drag_coeff * self.velocity
        thrust_force = np.array([0, 0, self.thrust])
        
        net_force = gravity_force + drag_force + thrust_force
        return net_force

    def update(self):
        net_force = self.compute_forces()
        acceleration = net_force / self.mass
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt

        # Check for ground collision
        if self.position[2] < 20:
            self.position[2] = 20
            self.velocity[2] = 0  # stop any downward motion

    def set_thrust(self, thrust_value):
        self.thrust = thrust_value

    def set_torque(self, roll, pitch, yaw):
        self.torque = np.array([roll, pitch, yaw])
        
    def run(self, num_steps):
        for _ in range(num_steps):
            self.update()
            print(f"Time: {self.dt * (_ + 1):.2f} sec | Position: {self.position}")

class DroneVisualizer:
    def __init__(self, drone_sim):
        self.drone_sim = drone_sim
        self.positions = [list(drone_sim.position)]
        
        # Pygame setup
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption('Drone Simulator')
        
        # Colors
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)

    def update(self):
        self.drone_sim.update()
        self.positions.append(list(self.drone_sim.position))

    def draw(self):
        self.screen.fill(self.WHITE)
        
        # Draw the ground at the equivalent of 10 meters
        ground_y_position = 600 - int(10 * 10)  # 10 meters * scaling factor
        pygame.draw.line(self.screen, (0, 0, 0), (0, ground_y_position), (800, ground_y_position), 5)

        # Draw the drone
        pygame.draw.circle(self.screen, self.RED, (400, 600 - int(self.drone_sim.position[2] * 10)), 5)  # Scale for better visualization
        pygame.display.flip()

        # Change the rendering to show vertical movement
        pygame.draw.circle(self.screen, self.RED, (400, 600 - int(self.drone_sim.position[2] * 10)), 5)  # Scale for better visualization
        pygame.display.flip()

    def plot_trajectory(self):
        positions = np.array(self.positions)
        plt.figure(figsize=(10,6))
        plt.plot(positions[:, 2], label='Z Position (Altitude)')
        plt.xlabel('Time step')
        plt.ylabel('Position')
        plt.title('Drone Altitude Over Time')
        plt.legend()
        plt.grid(True)
        plt.show()


    def run(self):
        clock = pygame.time.Clock()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:
                self.drone_sim.set_thrust(15)
            elif keys[pygame.K_s]:
                self.drone_sim.set_thrust(-15)
            else:
                self.drone_sim.set_thrust(0)

            self.update()
            self.draw()
            clock.tick(20)  # 20 FPS

        self.plot_trajectory()
        pygame.quit()

if __name__ == '__main__':
    drone_sim = DroneSimulator()
    visualizer = DroneVisualizer(drone_sim)
    visualizer.run()