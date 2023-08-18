import random
import math
import matplotlib.pyplot as plt
from matplotlib import gridspec

class Drone:
    def __init__(self):
        # Drone properties
        self.mass = 1.0
        self.thrust_coefficient = 0.1
        self.drag_coefficient = 0.01

        # State variables
        self.position = [0.0, 0.0, 0.0]
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

    def compute_forward_vector(self, pitch, yaw, roll):
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        fx = math.sin(yaw) * math.cos(pitch)
        fy = -math.cos(yaw) * math.cos(pitch)
        fz = math.sin(pitch)
        
        return fx, fy, fz

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

    def simulate_flight(self, actuator_sequences, durations):
        dt = 0.01
        total_time = sum(durations)
        num_steps = int(total_time / dt)
        x_data = []
        y_data = []
        z_data = []
        roll_data = []
        pitch_data = []
        yaw_data = []
        elapsed_time = 0

        vx_data = []
        vy_data = []
        vz_data = []
        omega_roll_data = []
        omega_pitch_data = []
        omega_yaw_data = []
        alpha_roll_data = []
        alpha_pitch_data = []
        alpha_yaw_data = []

        for step in range(num_steps):
            # Update actuator outputs based on elapsed time
            accumulated_duration = 0
            for idx, duration in enumerate(durations):
                accumulated_duration += duration
                if elapsed_time < accumulated_duration:
                    self.simulate(dt, actuator_sequences[idx])
                    break

            x_data.append(self.position[0])
            y_data.append(self.position[1])
            z_data.append(self.position[2])
            roll_data.append(self.orientation[0])
            pitch_data.append(self.orientation[1])
            yaw_data.append(self.orientation[2])
            elapsed_time += dt

            vx_data.append(self.velocity[0])
            vy_data.append(self.velocity[1])
            vz_data.append(self.velocity[2])
            omega_roll_data.append(self.angular_velocity[0])
            omega_pitch_data.append(self.angular_velocity[1])
            omega_yaw_data.append(self.angular_velocity[2])
            alpha_roll_data.append(self.angular_acceleration[0])
            alpha_pitch_data.append(self.angular_acceleration[1])
            alpha_yaw_data.append(self.angular_acceleration[2])

        return x_data, y_data, z_data, roll_data, pitch_data, yaw_data, vx_data, vy_data, vz_data, omega_roll_data, omega_pitch_data, omega_yaw_data, alpha_roll_data, alpha_pitch_data, alpha_yaw_data


def main():
    # Create a drone instance
    drone = Drone()

# Define simulation parameters
    simulation_time = 20  # seconds
    
    # Assuming position data every 0.01 seconds
    time_data = [i*0.01 for i in range(int(simulation_time/0.01))]

    hover_thrust_per_motor = 2.5 * drone.mass * 9.81  # Adjusted thrust for stability
    neutral_thrust_per_motor = 0

    # Define actuator outputs for each phase
    ascent_outputs = [hover_thrust_per_motor * 1.1] * 4  
    hover_outputs = [hover_thrust_per_motor] * 4
    descent_outputs = [hover_thrust_per_motor * 0.9] * 4

    # Create actuator sequences and durations lists
    actuator_sequences = [ascent_outputs, [20,10,0,50], ascent_outputs, [0,0,0,0]]
    durations = [5, 5, 5, 5]  # Assuming each phase lasts 5 seconds

    # Simulate the flight and get position and orientation data
    x_data, y_data, z_data, roll_data, pitch_data, yaw_data, vx_data, vy_data, vz_data, omega_roll_data, omega_pitch_data, omega_yaw_data, alpha_roll_data, alpha_pitch_data, alpha_yaw_data = drone.simulate_flight(actuator_sequences, durations)


    # Create a GridSpec layout
    fig = plt.figure(figsize=(12, 10))
    gs = gridspec.GridSpec(4, 2, width_ratios=[2, 1])

    # 3D plot for drone's movement
    ax1 = fig.add_subplot(gs[0, 0], projection='3d')
    ax1.plot(x_data, y_data, z_data)

    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Altitude (m)')
    ax1.set_title('Quadcopter 3D Flight Path')

    # Altitude plot
    ax5 = fig.add_subplot(gs[0, 1])
    ax5.plot(time_data, z_data)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Altitude (m)')
    ax5.set_title('Drone Altitude over Time')

    # Orientation data plots
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax5)
    ax2.plot(time_data, roll_data)
    ax2.set_ylabel('Roll (°)')
    ax2.set_title('Roll Orientation over Time')

    ax3 = fig.add_subplot(gs[1, 1], sharex=ax5)
    ax3.plot(time_data, pitch_data)
    ax3.set_ylabel('Pitch (°)')
    ax3.set_title('Pitch Orientation over Time')

    ax4 = fig.add_subplot(gs[2, 0], sharex=ax5)  # Use both columns for the yaw data
    ax4.plot(time_data, yaw_data)
    ax4.set_ylabel('Yaw (°)')
    ax4.set_title('Yaw Orientation over Time')

    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(time_data, vx_data, label="Vx")
    ax6.plot(time_data, vy_data, label="Vy")
    ax6.plot(time_data, vz_data, label="Vz")
    ax6.set_ylabel('Velocity (m/s)')
    ax6.set_title('Linear Velocity over Time')
    ax6.legend()

    ax7 = fig.add_subplot(gs[3, 0])
    ax7.plot(time_data, omega_roll_data, label="Roll")
    ax7.plot(time_data, omega_pitch_data, label="Pitch")
    ax7.plot(time_data, omega_yaw_data, label="Yaw")
    ax7.set_ylabel('Angular Velocity (rad/s)')
    ax7.set_title('Angular Velocity over Time')
    ax7.legend()

    ax8 = fig.add_subplot(gs[3, 1])
    ax8.plot(time_data, alpha_roll_data, label="Roll")
    ax8.plot(time_data, alpha_pitch_data, label="Pitch")
    ax8.plot(time_data, alpha_yaw_data, label="Yaw")
    ax8.set_ylabel('Angular Acceleration (rad/s^2)')
    ax8.set_title('Angular Acceleration over Time')
    ax8.legend()

    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    main()