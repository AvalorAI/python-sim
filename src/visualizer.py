class Drone3DMatplotlibVisualizer:
    def __init__(self, drone_sim):
        self.drone_sim = drone_sim
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.actuator_thrusts = [0, 0, 0, 0]

        self.trail, = self.ax.plot([], [], [], 'g:', linewidth=0.5)
        self.past_positions = []
    
        self.last_forward_quiver = None

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

        # Line representing drone orientation
        self.orientation_line, = self.ax.plot([0, 0], [0, 0], [0, 1], color='g')


    def update_plot(self):
        # Update drone position
        x, y, z = self.drone_sim.position
        self.drone_dot.set_data(x, y)
        self.drone_dot.set_3d_properties(z)
        
        # Fetch drone's current orientation as a rotation matrix
        orientation_matrix = R.from_quat(self.drone_sim.orientation).as_matrix()
        
        # Update actuator positions
        for actuator, dot in zip(self.drone_sim.actuators, self.actuator_dots):
            # Rotate each actuator's position with the orientation matrix
            rotated_position = orientation_matrix @ actuator.position
            x, y, z = self.drone_sim.position + rotated_position
            dot.set_data(x, y)
            dot.set_3d_properties(z)

        # Remove last forward vector quiver if it exists
        if self.last_forward_quiver:
            self.last_forward_quiver.remove()


        # Add a representation for drone's orientation
        forward_vector = np.array([1, 0, 0])  # Assuming drone's default forward is along x-axis

        # Rotate the forward vector using the drone's orientation
        orientation = R.from_quat(self.drone_sim.orientation)
        rotated_forward = orientation.apply(forward_vector)

        # Plot the rotated forward_vector as a quiver (arrow) from drone's current position
        self.last_forward_quiver = self.ax.quiver(self.drone_sim.position[0], self.drone_sim.position[1], self.drone_sim.position[2], *rotated_forward, length=5, normalize=True, color='r')

        self.past_positions.append(list(self.drone_sim.position))
        trail_x, trail_y, trail_z = zip(*self.past_positions)
        self.trail.set_data(trail_x, trail_y)
        self.trail.set_3d_properties(trail_z)

        # Assuming a fixed "view range" around the drone for all axes
        view_range = 10.0  
        
        x, y, z = self.drone_sim.position
        
        # Set the view limits dynamically based on the drone's position
        self.ax.set_xlim3d([x - view_range, x + view_range])
        self.ax.set_ylim3d([y - view_range, y + view_range])
        self.ax.set_zlim3d([z - view_range, z + view_range])

        plt.draw()
        plt.pause(0.005)

    def key_press_callback(self, event):
        key = event.key
        if key in ['1', '2', '3', '4']:
            index = int(key) - 1
            self.actuator_thrusts[index] = 100
            total_thrust = sum(self.actuator_thrusts)
            if total_thrust > 80:
                # Normalize thrusts if they exceed the limit
                factor = 80 / total_thrust
                self.actuator_thrusts = [thrust * factor for thrust in self.actuator_thrusts]
            self.drone_sim.set_actuator_thrusts(self.actuator_thrusts)

    def key_release_callback(self, event):
        key = event.key
        if key in ['1', '2', '3', '4']:
            index = int(key) - 1
            self.actuator_thrusts[index] = 0
            self.drone_sim.set_actuator_thrusts(self.actuator_thrusts)

    def run(self):
        # Bind the key press event
        self.fig.canvas.mpl_connect('key_press_event', self.key_press_callback)
        # Bind the key release event
        self.fig.canvas.mpl_connect('key_release_event', self.key_release_callback)
        
        while True:
            start_time = time.time()
            self.drone_sim.update() # <-- Adding this line here
            self.update_plot()
            elapsed_time = time.time() - start_time
            sleep_time = self.drone_sim.dt - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            if not plt.get_fignums():
                break
