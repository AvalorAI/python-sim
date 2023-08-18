import random
import math
import matplotlib.pyplot as plt

class Drone:
    def __init__(self):
        # Drone properties
        self.mass = 1.0
        self.thrust_coefficient = 0.1
        self.drag_coefficient = 0.01

        # State variables
        self.position = [0.0, 0.0, 30.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angular_acceleration = [0.0, 0.0, 0.0]

        # Control variables
        self.actuator_outputs = [0.0, 0.0, 0.0, 0.0]  # [front-left, front-right, rear-left, rear-right]

    def apply_thrust(self, actuator_outputs):
        self.actuator_outputs = actuator_outputs

    def compute_drag(self):
        velocity_magnitude = math.sqrt(sum(v ** 2 for v in self.velocity))
        drag_force = [-self.drag_coefficient * v * velocity_magnitude for v in self.velocity]
        return drag_force

    def compute_gravity(self):
        gravity_force = [0.0, 0.0, -self.mass * 9.81]
        return gravity_force

    def compute_net_force(self):
        total_thrust_magnitude = sum([self.thrust_coefficient * output for output in self.actuator_outputs])
        total_drag = self.compute_drag()
        total_gravity = self.compute_gravity()

        # Decompose thrust based on orientation (pitch, roll, yaw) and the new actuator setup
        thrust_x = total_thrust_magnitude * math.sin(self.orientation[1])  # sin(pitch)
        thrust_y = -total_thrust_magnitude * math.sin(self.orientation[0])  # -sin(roll)
        thrust_z = total_thrust_magnitude * math.cos(self.orientation[0]) * math.cos(self.orientation[1])  # cos(roll)*cos(pitch)

        
        net_force_x = thrust_x + total_drag[0]
        net_force_y = thrust_y + total_drag[1]
        net_force_z = thrust_z + total_drag[2] + total_gravity[2]
        
        return [net_force_x, net_force_y, net_force_z]

    def angular_accelerations(self):
        # Differential thrusts for roll, pitch, and yaw based on the new actuator setup
        roll_diff = (self.actuator_outputs[1] - self.actuator_outputs[0]) + (self.actuator_outputs[3] - self.actuator_outputs[2])
        pitch_diff = (self.actuator_outputs[0] + self.actuator_outputs[1]) - (self.actuator_outputs[2] + self.actuator_outputs[3])
        yaw_diff = (self.actuator_outputs[0] + self.actuator_outputs[3]) - (self.actuator_outputs[1] + self.actuator_outputs[2])

        roll_acceleration = roll_diff / self.mass
        pitch_acceleration = pitch_diff / self.mass
        yaw_acceleration = yaw_diff / self.mass

        return [roll_acceleration, pitch_acceleration, yaw_acceleration]

    def update_acceleration(self):
        self.acceleration = [force / self.mass for force in self.compute_net_force()]

    def update_velocity(self, dt):
        self.velocity = [v + a * dt for v, a in zip(self.velocity, self.acceleration)]

    def update_position(self, dt):
        self.position = [p + v * dt for p, v in zip(self.position, self.velocity)]

    def update_angular_acceleration(self):
        self.angular_acceleration = self.angular_accelerations()

    def update_angular_velocity(self, dt):
        self.angular_velocity = [w + alpha * dt for w, alpha in zip(self.angular_velocity, self.angular_acceleration)]

    def update_orientation(self, dt):
        self.orientation = [angle + omega * dt for angle, omega in zip(self.orientation, self.angular_velocity)]

    def simulate(self, dt, actuator_outputs):
        self.apply_thrust(actuator_outputs)
        self.update_acceleration()
        self.update_velocity(dt)
        self.update_position(dt)

        self.update_angular_acceleration()
        self.update_angular_velocity(dt)
        self.update_orientation(dt)

        # Print position and orientation for debugging
        print(f"Time: {round(dt, 2)}s")
        print(f"Position: X={round(self.position[0], 2)}m, Y={round(self.position[1], 2)}m, Z={round(self.position[2], 2)}m")
        print(f"Orientation: Roll={round(self.orientation[0], 2)}°, Pitch={round(self.orientation[1], 2)}°, Yaw={round(self.orientation[2], 2)}°")
        print("-----")

    def simulate_flight(self, time, actuator_outputs):
            dt = 0.01
            num_steps = int(time / dt)
            x_data = []
            y_data = []
            z_data = []
            roll_data = []
            pitch_data = []
            yaw_data = []
            for _ in range(num_steps):
                self.simulate(dt, actuator_outputs)
                x_data.append(self.position[0])
                y_data.append(self.position[1])
                z_data.append(self.position[2])
                roll_data.append(self.orientation[0])
                pitch_data.append(self.orientation[1])
                yaw_data.append(self.orientation[2])

            return x_data, y_data, z_data, roll_data, pitch_data, yaw_data

def main():
    # Create a drone instance
    drone = Drone()

    # Define simulation parameters
    simulation_time = 10  # seconds
    
    # Assuming position data every 0.01 seconds
    time_data = [i*0.01 for i in range(int(simulation_time/0.01))]

    hover_thrust_per_motor = 2.45 * drone.mass * 9.81  # Adjusted thrust for stability
    yaw_factor = 0.000001

    actuator_outputs = [
        hover_thrust_per_motor * (1.0),  # Front-left (CCW) increased
        hover_thrust_per_motor * (1.0),  # Front-right (CW) decreased
        hover_thrust_per_motor * (1.0),  # Rear-left (CCW) increased
        hover_thrust_per_motor * (1.05)   # Rear-right (CW) decreased
    ]

    # Simulate the flight and get position and orientation data
    x_data, y_data, z_data, roll_data, pitch_data, yaw_data = drone.simulate_flight(simulation_time, actuator_outputs)

    # 3D plot for drone's movement
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot(x_data, y_data, z_data)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Altitude (m)')
    ax1.set_title('Quadcopter 3D Flight Path')

    # Plotting the orientation data
    fig2, (ax2, ax3, ax4) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    ax2.plot(time_data, roll_data)
    ax2.set_ylabel('Roll (°)')
    ax2.set_title('Drone Orientation over Time')

    ax3.plot(time_data, pitch_data)
    ax3.set_ylabel('Pitch (°)')

    ax4.plot(time_data, yaw_data)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw (°)')

    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    main()