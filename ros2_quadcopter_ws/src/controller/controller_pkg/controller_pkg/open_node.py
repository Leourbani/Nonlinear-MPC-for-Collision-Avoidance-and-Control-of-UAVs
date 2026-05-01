import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
 
from time import time

from tf_transformations import euler_from_quaternion

import numpy as np
import opengen as og
import matplotlib.pyplot as plt
import sys # for sys.exit in case of big error
from matplotlib.animation import FuncAnimation # for animations



# with args, run:
# ros2 run controller_pkg open_node --ros-args -p trajectory:=2


class OpenNode(Node):
    def __init__(self):
        super().__init__('open_node')

        """
        Trajectory choice:
        0 -- static object
        1 -- linear motion
        2 -- parabolic motion
        """
        #obstacle trajectory:0 = static, 1 = linear, 2 = projectile
        self.declare_parameter('trajectory', 1) 
        self.real_traj_number = self.get_parameter('trajectory').get_parameter_value().integer_value

        # mode: 0=nothing, 1=graphs, 2=graphs+animation
        self.declare_parameter('mode', 2) 
        self.mode = self.get_parameter('mode').get_parameter_value().integer_value
        
        self.T_sim = 20.0  # Total simulation time Total sim[seconds]

        # ===== Configuration and Global Parameters =====
        self.optimizer_path = 'build/drone_controller'

        # Dimensions of the problem MUST correspond with the ones used in the solver generator
        self.nx = 8  # State
        self.nu = 3  # Control Input
        self.N = 40  # Number of predictive intervals
        self.dt = 0.05
        self.M=5 # backwards prediction horizon
        self.n_simulation_steps = int(self.T_sim / self.dt)


        # Drone Dynamics Parameters
        self.drone_dyn_params = {
            'Ax': 0.1, 'Ay': 0.1, 'Az': 0.2,
            'g': 9.81,
            'tau_phi': 0.5, 'K_phi': 1.0,
            'tau_theta': 0.5, 'K_theta': 1.0,
            'Bx': 0.03, 'By': 0.03, 'Bz':0.06 
        }
        self.m = 0.225 
        self.dt = 0.05
        self.c = 1.0 # to calibrate if needed -> used in conversion from torque to velocity



        # Obstacle Parameters (according to trajectory choice)
        # self.p0_obs_initial (initial obstacle pose) must be the same as in the launch 
        if self.real_traj_number == 0:
            self.vel_obs_initial = np.array([0.0, 0.0, 0.0]) # Still obstacle
            self.p0_obs_initial = np.array([4.0, 4.0, 4.0])
        elif self.real_traj_number == 1:
            #self.vel_obs_initial = np.array([-2.0, -2.0, 0.0]) # Constant speed of obstacle
            #self.p0_obs_initial = np.array([4.0, 4.0, 4.0])
            self.vel_obs_initial = np.array([-2.0, 0.0, 0.0]) # Constant speed of obstacle
            self.p0_obs_initial = np.array([15.0, 0.0, 2.0])
        elif self.real_traj_number == 2:
            self.p0_obs_initial = np.array([15.0, 15.0, 4.0]) 
            self.pf_obs_final = np.array([0.5, 0.5, 0.5]) # Final position
            self.T = 3.0  # deisred time to reach final position

            # end in the origin:
            # self.vx0 = - self.p0_obs_initial[0] * self.drone_dyn_params['Bx'] / (1 - np.exp(-self.drone_dyn_params['Bx'] * self.T))
            # self.vy0 = - self.p0_obs_initial[1] * self.drone_dyn_params['By'] / (1 - np.exp(-self.drone_dyn_params['By'] * self.T))
            # self.vz0 = self.drone_dyn_params['Bz'] * (-self.p0_obs_initial[2] + (self.drone_dyn_params['g'] / self.drone_dyn_params['Bz']) * self.T) / (1 - np.exp(-self.drone_dyn_params['Bz'] * self.T)) - self.drone_dyn_params['g'] / self.drone_dyn_params['Bz']
            
            # end in a different chosen point:
            self.vx0 = self.drone_dyn_params['Bx'] * ((self.pf_obs_final[0] - self.p0_obs_initial[0]) / (1 - np.exp(-self.drone_dyn_params['Bx'] * self.T)) )
            self.vy0 = self.drone_dyn_params['By'] * ((self.pf_obs_final[1] - self.p0_obs_initial[1]) / (1 - np.exp(-self.drone_dyn_params['By'] * self.T)) )
            self.vz0 = self.drone_dyn_params['Bz'] * ((self.pf_obs_final[2] - self.p0_obs_initial[2] + (self.drone_dyn_params['g'] / self.drone_dyn_params['Bz']) * self.T) / (1 - np.exp(-self.drone_dyn_params['Bz'] * self.T))) - (self.drone_dyn_params['g'] / self.drone_dyn_params['Bz'])
            
            self.vel_obs_initial = np.array([self.vx0, self.vy0, self.vz0])
            

        self.r_obs_val = np.array([0.3]) # Radius obstacle

        # Initial drone's state and previous input (start with all zeros)
        self.x0_initial_drone = np.array([0.0, 0.0, 0.0,  # Position: px, py, pz
                                          0.0, 0.0, 0.0,  # Velocity: vx, vy, vz
                                          0.0, 0.0])      # Angles: phi, theta
        self.current_x0_drone = self.x0_initial_drone
        self.u_prev_initial = np.array([self.drone_dyn_params['g'], 0.0, 0.0]) # Input for hovering (Thrust, phi_ref, theta_ref)


        # ===== REFERENCE TRAJECTORY (FOR THE FULL N+1 HORIZON) =====
        self.x_ref_target_single = np.array([0.0, 5.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        # solver wants N+1 reference states
        self.x_ref_seq_horizon = np.tile(self.x_ref_target_single, (self.N + 1)) # Vector length = nx * (N+1)



        # ===== SERVER CONNECTION =====
        print("Trying to connect to the optimizer's server..")
        try:
            self.mng = og.tcp.OptimizerTcpManager(self.optimizer_path)
            self.mng.ping() # Verifying connection
            print("Server connected properly.")
        except Exception as e:
            print(f"Error: Impossible to connect to optimizer's server in '{self.optimizer_path}'.")
            print(f"Error details: {e}")
            print("Ensure that the server is being executed.")
            sys.exit(1)



        # ===== SIMULATION LOOP ===== 

        # Array to memories states history
        self.drone_states_history = np.zeros((self.n_simulation_steps + 1, self.nx)) # drone's states
        self.applied_inputs_history = np.zeros((self.n_simulation_steps, self.nu)) # inputs applied
        self.obs_pos_history = np.zeros((self.n_simulation_steps + 1, 3)) # only obstacle position
        self.obs_velocity_history = np.zeros((self.n_simulation_steps + 1, 3)) # only obstacle velocity
        self.obs_states_history = np.zeros((self.n_simulation_steps + 1, 6)) # full state of the obstacle [p, v]

        # States initialization
        self.drone_states_history[0, :] = self.x0_initial_drone
        self.obs_pos_history[0, :] = self.p0_obs_initial
        self.current_u_prev = np.copy(self.u_prev_initial)
        self.obs_states_history[0, :3] = self.p0_obs_initial
        self.current_vel_obs = self.vel_obs_initial
        self.obs_velocity_history[0, :] = self.current_vel_obs
        self.obs_states_history[0, -3:] = self.current_vel_obs


        self.actual_input_to_apply = None


        # ===================================================
        # =================== DRONE ROS2 ====================
        # ===================================================

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.odom_subscription = self.create_subscription(Odometry, '/simple_quad/odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.start_time = time()
        self.start_time_step = time()
        self.k_step = 0

        self.flag_end = False
        self.flag_graph = False

        self.current_drone_pose = self.x0_initial_drone




        # ===================================================
        # ==================== SPHERE =======================
        # ===================================================

        self.sphere_current_position = self.p0_obs_initial
        self.sphere_current_velocity = self.vel_obs_initial
        self.sphere_kp = 2.5 # Guadagno del controllore proporzionale (puoi regolarlo)

        # publisher to sent velocity commands to the sphere
        self.sphere_velocity_publisher = self.create_publisher(Twist, '/sphere_cmd_vel', 10)
        # subscriber to the actual sphere position from odometry topic
        self.sphere_odom_subscriber = self.create_subscription(Odometry, '/sphere_odom', self.sphere_odom_callback, 10)

        # timer to control the sphere functions
        self.sphere_control_timer = self.create_timer(self.dt, self.sphere_control_loop)









#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################

    # ===================================================
    # ======== TRAJECTORY PREDICTION FUNCTIONS ==========
    # ===================================================
    # for this operation we need the previous 'M' measurements of position and velocity of the object.
    # so, we start doing the trajectory classification after 5 time istants
    # in our simulation we use the object trajectory we calculated until now and we store them in the array 'recorded_measurements'

    #  --- 3 functions to compute the expected measurements for each trajectory ---
    def parabolic_traj_prev(self, p, v, M, pvals_dyn):
        """
        The function returns an array of M previous obstacle states predicted with parabolic motion
        p -- last position
        v -- last velocity
        M -- length of backward prediction
        """
        x_previous_array_p=np.zeros((self.M, 6))

        for i in range(self.M):
            x_previous_array_p[4-i, 3] = v[0] * np.exp(pvals_dyn['Bx'] * self.dt * i)
            x_previous_array_p[4-i, 4] = v[1] * np.exp(pvals_dyn['By'] * self.dt * i)
            x_previous_array_p[4-i, 5] = (v[2] + pvals_dyn['g'] / pvals_dyn['Bz']) * np.exp(pvals_dyn['Bz'] * self.dt * i) - pvals_dyn['g'] / pvals_dyn['Bz']
            x_previous_array_p[4-i, 0]= p[0] - x_previous_array_p[4-i, 3] / pvals_dyn['Bx'] * (1-np.exp(-pvals_dyn['Bx'] * self.dt * i))
            x_previous_array_p[4-i, 1]= p[1] - x_previous_array_p[4-i, 4] / pvals_dyn['By'] * (1-np.exp(-pvals_dyn['By'] * self.dt * i))
            x_previous_array_p[4-i, 2] = p[2] - (x_previous_array_p[4-i, 5] + pvals_dyn['g'] / pvals_dyn['Bz']) * (1 - np.exp(-pvals_dyn['Bz'] * self.dt * i)) / pvals_dyn['Bz'] + (pvals_dyn['g'] / pvals_dyn['Bz']) * self.dt * i
        return x_previous_array_p

    def linear_traj_prev(self, p, v, M):
        """
        The function returns an array of M previous obstacle states predicted with linear motion
        p -- last position
        v -- last velocity
        M -- length of backward prediction
        """
        x_previous_array_l=np.zeros((self.M, 6))
        for i in range(self.M):
            x_previous_array_l[4-i] = np.concatenate([p - i*self.dt*v, v])
        return x_previous_array_l

    def static_traj_prev(self, p, v, M):
        """
        The function returns an array of M previous obstacle states predicted with static motion
        p -- last position
        v -- last velocity
        M -- length of backward prediction
        """
        x_previous_array_s=np.zeros((self.M, 6))
        x_previous_array_s[:]=np.concatenate([p, v])
        return x_previous_array_s

    # trajectory prediction
    def traj_prediction(self, recorded_measurements, obs_p, obs_v, M, pvals_dyn):
        """
        Returns the parameter 'traj_number' according to the trajectory with less error:
        traj_error:
        0 -- static trajectory
        1 -- linear trajectory
        2 -- parabolic trajectory
        """
        # prediction according to the different 
        x_previous_array_s = self.static_traj_prev(obs_p, obs_v, self.M)
        x_previous_array_l = self.linear_traj_prev(obs_p, obs_v, self.M)
        x_previous_array_p = self.parabolic_traj_prev(obs_p, obs_v, self.M, pvals_dyn)

        # error
        traj_err_s=float(np.sum(abs(x_previous_array_s[:, :3] - recorded_measurements[-5:, -6:-3]) + abs(x_previous_array_s[:, -3:]-recorded_measurements[-5:, -3:])))
        traj_err_l=float(np.sum(abs(x_previous_array_l[:, :3] - recorded_measurements[-5:, -6:-3]) + abs(x_previous_array_l[:, -3:]-recorded_measurements[-5:, -3:])))
        traj_err_p=float(np.sum(abs(x_previous_array_p[:, :3] - recorded_measurements[-5:, -6:-3]) + abs(x_previous_array_p[:, -3:]-recorded_measurements[-5:, -3:])))
        # debug 
        # print(f"Static / linear / parabolic error: {traj_err_s} / {traj_err_l} / {traj_err_p}")

        traj_number = min(range(3), key=lambda i:[traj_err_s, traj_err_l, traj_err_p][i]) # controllare

        return traj_number







#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################



    # ===================================================
    # ===================== DRONE =======================
    # ===================================================

    def odom_callback(self, msg):
        """
        DRONE: Callback that extracts position and linear velocity from odometry and saves them
        """
        position = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation
        q = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        # angular = msg.twist.twist.angular

        px = position.x
        py = position.y
        pz = position.z
        # qx = orientation.x
        # qy = orientation.y
        # qz = orientation.z
        # qw = orientation.w
        # roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)    
        vx = linear.x
        vy = linear.y
        vz = linear.z

        self.current_x0_drone = np.array([px, py, pz,  # Position: px, py, pz
                                          vx, vy, vz,  # Velocity: vx, vy, vz
                                          roll, pitch])      # Angles: phi, theta
        self.current_drone_pose[:6] = np.array([px, py, pz,  # Position: px, py, pz
                                              vx, vy, vz])  # Velocity: vx, vy, vz
        
        if self.flag_end == False:
            self.get_logger().info(f'Odom drone received - Position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})')
        # self.get_logger().info(
        #     f'Odom received - Position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), '
        #     f'Linear vel: ({linear.x:.2f}, {linear.y:.2f}, {linear.z:.2f}),'
        #     f'Orientation: ({roll:.2f}, {pitch:.2f})')




    def timer_callback(self):
        
        time_step = time()

        if time_step-self.start_time < self.T_sim:
        # for k_step in range(self.n_simulation_steps):
            # print(f"\n--- Simulation iteration {k_step + 1}/{self.n_simulation_steps} ---")

            """ STEP (1)
            1. Gets the current state of the drone and the current state of the obstacle 
            """
            self.current_x0_drone = self.current_drone_pose
            self.current_p_obs_sim = self.sphere_current_position
            self.current_vel_obs =  self.sphere_current_velocity

            # save drone state in this step
            self.drone_states_history[self.k_step+1, :] = self.current_x0_drone
            
            # save obstacol state in this step
            self.obs_pos_history[self.k_step+1, :] = self.current_p_obs_sim
            self.obs_velocity_history[self.k_step+1, :] = self.sphere_current_velocity
            self.obs_states_history[self.k_step+1, :3] = self.current_p_obs_sim
            self.obs_states_history[self.k_step+1, -3:] = self.sphere_current_velocity
            
            
            """ STEP (2)
            2. Investigate the obstacle trajectory using the last positions and velocities measured
            """
            if self.k_step<=self.M:
                est_traj_number=self.real_traj_number # at first, we suggest the correct obstacle movement
            if self.k_step>self.M: # M sates need to be ready
                est_traj_number=self.traj_prediction(self.obs_states_history[:self.k_step+1], self.current_p_obs_sim, self.current_vel_obs, self.M, self.drone_dyn_params) #problema con vel_obs_const...
                # if est_traj_number== 1:
                #     print(f"Iterazione numero {self.k_step+1} individua traiettoria lineare")
                # if est_traj_number == 2:
                #     print(f"Iterazione numero {self.k_step+1} individua traiettoria parabolica")
                # if est_traj_number==0:
                #     print(f"Iterazione numero {self.k_step+1} individua traiettoria statica")
            
            """
            STEP (3)
            3. Prepares the parameter vector `p_runtime` for the solver
                The order MUST correspond to the one defined on solver
                Assuming as order: x0, u_prev, x_ref_horizon (N+1 stati), r_obs, p_obs_initial, vel_obs
            """
            p_runtime = np.concatenate([
                self.current_x0_drone,             # (nx,)
                self.current_u_prev,               # (nu,)
                self.x_ref_seq_horizon.flatten(),  # (nx * (N+1),) -> Ensure that x_ref_seq_horizon is correct
                self.r_obs_val,                    # (1,)
                self.current_p_obs_sim,            # (3,) -> Actual obstacle position, used as initial for the solver prediction
                self.current_vel_obs,              # (3,)
                np.array([est_traj_number])       # we tell the solver which trajectory to use to simulate the object
            ])


            """ STEP (4)
            4. Solving the problem:
                - calling the solver
                - processing solver solution
            """
            solution = self.mng.call(p_runtime.tolist())
            
            if solution.is_ok():
                solution_data = solution.get()
                u_star_optimal_sequence = np.array(solution_data.solution) # From float list to NuMpy array
                # Verifying that u_star_optimal_sequence has the expected dimension nu*N
                if len(u_star_optimal_sequence) == self.nu * self.N:
                    u_star_reshaped = np.reshape(u_star_optimal_sequence, (self.N, self.nu))
                    self.actual_input_to_apply = u_star_reshaped[0, :] # apply the first input of the optimal sequence
                    # print(f"Soluzione NMPC trovata. Input da applicare: {self.actual_input_to_apply}")
                else:
                    print(f"  ERROR: Solver dimension is unexpected ({len(u_star_optimal_sequence)}), expected ({self.nu*self.N}).")
                    self.actual_input_to_apply = self.current_u_prev # Fallback
                    print(f"  Using previous input because of error dimension: {self.actual_input_to_apply}")
            else:
                solver_error = solution.get()
                error_code = solver_error.code
                error_msg = solver_error.message
                print(f"  ERROR from NMPC solver! Code: {error_code}, Message: {error_msg}")
                self.actual_input_to_apply = self.current_u_prev # Fallback
                print(f"  Using previous input because of error dimension: {self.actual_input_to_apply}")


            """ STEP (5)
            5. Using selected input for the drone model
            """
            if isinstance(self.actual_input_to_apply, np.ndarray):
                self.applied_inputs_history[self.k_step, :] = self.actual_input_to_apply
            else:
                print(f"CRITICAL ERROR: No valid input to apply to the iteration {self.k_step}. Interrupting.")
                # break # In case we want interrupt the simulation


            """ STEP (6)
            6. Update u_prev for the next server call
            """
            self.current_u_prev = self.actual_input_to_apply

            # update number of step
            self.k_step = self.k_step+1

        # if simulation time reaches the end
        elif self.flag_end == False:
            self.flag_end = True
            print("\nSimulation completed.")
            self.actual_input_to_apply[0]=0 
            self.actual_input_to_apply[1]=0 
            self.actual_input_to_apply[2]=0

        T = self.actual_input_to_apply[0] 
        phi_ref = self.actual_input_to_apply[1] 
        theta_ref = self.actual_input_to_apply[2]

        twist_msg = Twist()
        #twist_msg.linear.x = - velxy * np.sin(theta_ref)
        #twist_msg.linear.y = velxy * np.cos(theta_ref) * np.sin(phi_ref)
        #twist_msg.linear.z = velz * np.cos(theta_ref) * np.cos(phi_ref)
        twist_msg.linear.x = - T * np.sin(theta_ref) * self.c*self.dt/self.m
        twist_msg.linear.y = T * np.cos(theta_ref) * np.sin(phi_ref) * self.c*self.dt/self.m
        twist_msg.linear.z = (T * np.cos(theta_ref) * np.cos(phi_ref)* self.c - 9.81) *self.dt/self.m
        
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)

        # if self.flag_end == False:
        #     self.get_logger().info(
        #         f"Vx={twist_msg.linear.x:.2f} m/s, "
        #         f"Vy={twist_msg.linear.y:.2f} m/s, "
        #         f"Vz={twist_msg.linear.z:.2f} m/s",
        #     )
        # if self.flag_end == False:
        #     self.get_logger().info(
        #         f"T={self.actual_input_to_apply[0]:.2f} (m/s^2), "
        #         # f"phi_ref={math.degrees(self.actual_input_to_apply[1]):.2f} deg, "
        #         # f"theta_ref={math.degrees(self.actual_input_to_apply[2]):.2f} deg",
        #     )

        if self.flag_end == True and self.flag_graph == False:
            self.flag_graph = True
            if self.mode == 1:
                self.plot()
            elif self.mode == 2:
                self.plot()
                self.plot_animation()





#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################



    # ===================================================
    # ==================== SPHERE =======================
    # ===================================================

    def sphere_odom_callback(self, msg):
        """
        SPHERE: Callback that extracts position and linear velocity from odometry and saves them
        """
        pos = msg.pose.pose.position
        self.sphere_current_position = np.array([pos.x, pos.y, pos.z])

        vel = msg.twist.twist.linear
        self.sphere_current_velocity = np.array([vel.x, vel.y, vel.z])
        
        #if self.flag_end == False:
         #   self.get_logger().info(
          #      f"Sphere - Pos: {self.sphere_current_position}, Vel: {self.sphere_current_velocity}"
           # )



    def sphere_control_loop(self):
        """
        SPHERE: Publisher of the sphere velocities, according to the type of trajectory choosen
        """       
        current_time = time() # to evaluate more precisely the duration of the step
        dt = current_time - self.start_time_step

        dt_sphere = time() - self.start_time

        if self.flag_end == True or (self.k_step < self.M and self.sphere_current_position[2] < 0.2):
            command_velocity = np.array([0.0, 0.0, 0.0])
        elif self.real_traj_number == 0:
            command_velocity = self.vel_obs_initial
        elif self.real_traj_number == 1:
            command_velocity = self.vel_obs_initial
        elif self.real_traj_number==2:
            # PROJECTILE
            if self.k_step < self.M:
                vx_new = self.vel_obs_initial[0]
                vy_new = self.vel_obs_initial[1]
                vz_new = self.vel_obs_initial[2] 
            else:
                # # con origine
                # vx_new = self.current_vel_obs[0] * np.exp(-self.drone_dyn_params['Bx'] * dt)
                # vy_new = self.current_vel_obs[1] * np.exp(-self.drone_dyn_params['By'] * dt)
                # vz_new = ((self.current_vel_obs[2] + self.drone_dyn_params['g'] / self.drone_dyn_params['Bz']) * np.exp(-self.drone_dyn_params['Bz'] * dt) - self.drone_dyn_params['g'] / self.drone_dyn_params['Bz'])
                
                vx_new = self.vel_obs_initial[0] * np.exp(-self.drone_dyn_params['Bx'] * dt_sphere)
                vy_new = self.vel_obs_initial[1] * np.exp(-self.drone_dyn_params['By'] * dt_sphere)
                vz_new = ((self.vel_obs_initial[2] + self.drone_dyn_params['g'] / self.drone_dyn_params['Bz']) * np.exp(-self.drone_dyn_params['Bz'] * dt_sphere) - self.drone_dyn_params['g'] / self.drone_dyn_params['Bz'])
                
            command_velocity = np.array([vx_new, vy_new, vz_new]) # just in case of parabolic trajectory the velocity changes

        # message creation
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = command_velocity[0]
        cmd_vel_msg.linear.y = command_velocity[1]
        cmd_vel_msg.linear.z = command_velocity[2] 
        
        self.start_time_step = time() #update of start time for the next step       

        # publish message
        self.sphere_velocity_publisher.publish(cmd_vel_msg)
        
        # if self.flag_end == False:
        #     self.get_logger().info(
        #         f"Vx={cmd_vel_msg.linear.x:.2f} m/s, "
        #         f"Vy={cmd_vel_msg.linear.y:.2f} m/s, "
        #         f"Vz={cmd_vel_msg.linear.z:.2f} m/s",
        #     )




#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################

# ============================
# ===== RESULTS PLOTTING =====
# ============================

    def plot(self):
        print("Graphics generation...")

        # Extracting trajectories for the plot
        self.drone_pos_history = self.drone_states_history[:, 0:3]
        self.obs_pos_history_plot = self.obs_pos_history[:, 0:3]

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot drone trajectory
        ax.plot(self.drone_pos_history[:, 0], self.drone_pos_history[:, 1], self.drone_pos_history[:, 2],
                label='Traiettoria Drone', linewidth=2, color='blue')
        ax.scatter(self.drone_pos_history[0, 0], self.drone_pos_history[0, 1], self.drone_pos_history[0, 2],
                color='green', s=100, label='Start Drone', marker='o')
        ax.scatter(self.drone_pos_history[-1, 0], self.drone_pos_history[-1, 1], self.drone_pos_history[-1, 2],
                color='red', s=100, label='End Drone', marker='X')

        # Plot obstacle trajectory
        ax.plot(self.obs_pos_history_plot[:, 0], self.obs_pos_history_plot[:, 1], self.obs_pos_history_plot[:, 2],
                label='Traiettoria Ostacolo', linewidth=2, color='orange', linestyle='--')
        ax.scatter(self.obs_pos_history_plot[0, 0], self.obs_pos_history_plot[0, 1], self.obs_pos_history_plot[0, 2],
                color='magenta', s=80, label='Start Ostacolo', marker='s')

        # Plot ref point (if fixed)
        ax.scatter(self.x_ref_target_single[0], self.x_ref_target_single[1], self.x_ref_target_single[2],
                color='black', s=120, label='Setpoint Riferimento', marker='P')


        # To better read the plot
        max_range = np.array([self.drone_pos_history[:, 0].max() - self.drone_pos_history[:, 0].min(),
                            self.drone_pos_history[:, 1].max() - self.drone_pos_history[:, 1].min(),
                            self.drone_pos_history[:, 2].max() - self.drone_pos_history[:, 2].min()]).max() / 2.0
        if max_range < 0.1: max_range = 1.0 # Avoid too small ranges if the drone doesn't moove

        mid_x = (self.drone_pos_history[:, 0].max() + self.drone_pos_history[:, 0].min()) * 0.5
        mid_y = (self.drone_pos_history[:, 1].max() + self.drone_pos_history[:, 1].min()) * 0.5
        mid_z = (self.drone_pos_history[:, 2].max() + self.drone_pos_history[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('3D Trajectory Simulation Drone NMPC')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()

        # Control input plots
        fig_inputs, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        time_vector = np.arange(self.n_simulation_steps) * self.dt

        axs[0].plot(time_vector, self.applied_inputs_history[:, 0], label='Thrust (T)')
        axs[0].set_ylabel('Thrust [N]')
        axs[0].grid(True)
        axs[0].legend()

        axs[1].plot(time_vector, self.applied_inputs_history[:, 1], label='Phi Ref (roll)')
        axs[1].set_ylabel('Phi Ref [rad]')
        axs[1].grid(True)
        axs[1].legend()

        axs[2].plot(time_vector, self.applied_inputs_history[:, 2], label='Theta Ref (pitch)')
        axs[2].set_ylabel('Theta Ref [rad]')
        axs[2].set_xlabel('Tempo [s]')
        axs[2].grid(True)
        axs[2].legend()

        fig_inputs.suptitle('Input di Controllo Applicati al Drone')
        plt.tight_layout(rect=[0, 0, 1, 0.96])

        # Drone states plot
        fig_states, axs_s = plt.subplots(2, 1, sharex=True, figsize=(10,8))
        axs_s[0].plot(time_vector, self.drone_states_history[:-1, 0], label='Posizione X')
        axs_s[0].plot(time_vector, self.drone_states_history[:-1, 1], label='Posizione Y')
        axs_s[0].plot(time_vector, self.drone_states_history[:-1, 2], label='Posizione Z')
        axs_s[0].set_ylabel('Posizione [m]')
        axs_s[0].grid(True)
        axs_s[0].legend()

        axs_s[1].plot(time_vector, self.drone_states_history[:-1, 3], label='Velocità Vx')
        axs_s[1].plot(time_vector, self.drone_states_history[:-1, 4], label='Velocità Vy')
        axs_s[1].plot(time_vector, self.drone_states_history[:-1, 5], label='Velocità Vz')
        axs_s[1].set_ylabel('Velocità [m/s]')
        axs_s[1].set_xlabel('Tempo [s]')
        axs_s[1].grid(True)
        axs_s[1].legend()

        fig_states.suptitle('Stati Selezionati del Drone')
        plt.tight_layout(rect=[0, 0, 1, 0.96])


        plt.show()

        # print("Dimensione p_runtime:", len(self.p_runtime))
        # '\n'
        # print('Dovrebbe essere uguale al numero di parametri del solver generator')
        #print("Prime 10 componenti di p_runtime:", p_runtime[:10])






    # =========================
    # ===== 3D ANIMATION ======
    # =========================

    # Initialize animation function
    def init_animation(self):
        self.line_drone.set_data([], [])
        self.line_drone.set_3d_properties([])
        self.marker_drone.set_data([], [])
        self.marker_drone.set_3d_properties([])

        self.line_obs.set_data([], [])
        self.line_obs.set_3d_properties([])
        self.marker_obs.set_data([], [])
        self.marker_obs.set_3d_properties([])
        
        return self.line_drone, self.marker_drone, self.line_obs, self.marker_obs,

    # Update animation function (called for each frame)
    def update_animation(self, frame_idx):
        # drone trace update
        self.line_drone.set_data(self.drone_pos_history[:frame_idx+1, 0], self.drone_pos_history[:frame_idx+1, 1])
        self.line_drone.set_3d_properties(self.drone_pos_history[:frame_idx+1, 2])
        # drone marker postion update
        self.marker_drone.set_data([self.drone_pos_history[frame_idx, 0]], [self.drone_pos_history[frame_idx, 1]])
        self.marker_drone.set_3d_properties([self.drone_pos_history[frame_idx, 2]])

        # obstacle trace udpate
        self.line_obs.set_data(self.obs_pos_history_plot[:frame_idx+1, 0], self.obs_pos_history_plot[:frame_idx+1, 1])
        self.line_obs.set_3d_properties(self.obs_pos_history_plot[:frame_idx+1, 2])
        # obsatcle marker position update
        self.marker_obs.set_data([self.obs_pos_history_plot[frame_idx, 0]], [self.obs_pos_history_plot[frame_idx, 1]])
        self.marker_obs.set_3d_properties([self.obs_pos_history_plot[frame_idx, 2]])

        self.ax_anim.set_title(f'3D trajectory animation - Time: {frame_idx*self.dt:.2f} s')
        return self.line_drone, self.marker_drone, self.line_obs, self.marker_obs,




    def plot_animation(self):
        print("Generating 3D animation...")

        self.fig_anim = plt.figure(figsize=(12, 9))
        self.ax_anim = self.fig_anim.add_subplot(111, projection='3d')

        # axis limits, based on whole trajectory
        all_x = np.concatenate((self.drone_pos_history[:, 0], self.obs_pos_history_plot[:, 0]))
        all_y = np.concatenate((self.drone_pos_history[:, 1], self.obs_pos_history_plot[:, 1]))
        all_z = np.concatenate((self.drone_pos_history[:, 2], self.obs_pos_history_plot[:, 2]))

        max_range_anim = np.array([all_x.max() - all_x.min(),
                                all_y.max() - all_y.min(),
                                all_z.max() - all_z.min()]).max() / 1.8 # Denominator to zoom a little less
        if max_range_anim < 0.5: max_range_anim = 1.0

        mid_x_anim = (all_x.max() + all_x.min()) * 0.5
        mid_y_anim = (all_y.max() + all_y.min()) * 0.5
        mid_z_anim = (all_z.max() + all_z.min()) * 0.5

        self.ax_anim.set_xlim(mid_x_anim - max_range_anim, mid_x_anim + max_range_anim)
        self.ax_anim.set_ylim(mid_y_anim - max_range_anim, mid_y_anim + max_range_anim)
        self.ax_anim.set_zlim(mid_z_anim - max_range_anim, mid_z_anim + max_range_anim) # to include obsatcle

        self.ax_anim.set_xlabel('X [m]')
        self.ax_anim.set_ylabel('Y [m]')
        self.ax_anim.set_zlabel('Z [m]')
        self.ax_anim.set_title('Animazione Traiettoria 3D NMPC')

        # Oggetti grafici per l'animazione
        # Linee per le tracce complete (disegnate una volta come riferimento opzionale)
        # ax_anim.plot(drone_pos_history[:, 0], drone_pos_history[:, 1], drone_pos_history[:, 2], 'b:', linewidth=1, label='Traiettoria Completa Drone')
        # ax_anim.plot(self.obs_pos_history_plot[:, 0], self.obs_pos_history_plot[:, 1], self.obs_pos_history_plot[:, 2], color='orange', linestyle=':', linewidth=1, label='Traiettoria Completa Ostacolo')

        # Lines drawn each frame
        self.line_drone, = self.ax_anim.plot([], [], [], 'b-', linewidth=2, label='Traiettoria Drone')
        self.line_obs, = self.ax_anim.plot([], [], [], color='orange', linestyle='--', linewidth=2, label='Traiettoria Ostacolo')

        # Markers for current position
        self.marker_drone, = self.ax_anim.plot([], [], [], color='blue', marker='o', markersize=8, label='Drone')
        self.marker_obs, = self.ax_anim.plot([], [], [], color='red', marker='s', markersize=8, label='Ostacolo')

        # Reference point (if fixed)
        self.ax_anim.scatter(self.x_ref_target_single[0], self.x_ref_target_single[1], self.x_ref_target_single[2],
                    color='black', s=100, label='Setpoint Riferimento', marker='P', depthshade=True)


        # Creation of animation
        # n_simulation_steps + 1: number of states (frames)
        # interval: interval between frames [ms] (dt * 1000 for real time)
        ani = FuncAnimation(self.fig_anim, self.update_animation, frames=self.n_simulation_steps + 1,
                            init_func=self.init_animation, blit=False, interval=self.dt*500, repeat=False)

        self.ax_anim.legend(loc='upper right')
        plt.tight_layout()

        # Per salvare l'animazione 
        # print("Saving animation...")
        # try:
        #     ani.save('simulazione_traiettoria.mp4', writer='ffmpeg', fps=int(1/dt), dpi=150)
        #     print("Animazione salvata come simulazione_traiettoria.mp4")
        # except Exception as e:
        #     print(f"Errore nel salvataggio dell'animazione: {e}")
        #     print("Assicurati che ffmpeg sia installato e nel PATH di sistema.")



        plt.show() # show figures and animations



#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
#######################################################################################################################




def main(args=None):
    rclpy.init(args=args)
    node = OpenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Chiusura nodo per KeyboardInterrupt.")
    finally:
        if rclpy.ok(): 
            node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()






