import casadi.casadi as cs
import opengen as og
import opengen.constraints as ogc
import math as mt
import numpy as np

# === Dimensions ===
nu = 3
nx = 8
N = 40  # Number of control intervals. This means N inputs u_0, ..., u_{N-1}
        # and N+1 states x_0, ..., x_N
dt = 0.05
dphi_max = 0.08
dtheta_max = 0.08

# === Symbolic Variables ===
# u_decision_vars represents N control inputs: u_0, u_1, ..., u_{N-1}
u_decision_vars = cs.SX.sym("u_decision_vars", nu * N)

# Parametrers p:
# x0_p: initial state (nx)
# u_prev_p: previous control input (nu)
# x_ref_horizon_p: desired trajectory for N+1 states (da x_ref_0 a x_ref_N) (nx * (N+1))
# r_obs_p: obstacle radius (1)
# p_obs_initial_p: obstacle initial position (3)
# vel_obs_p: obstacle constant speed (3)
# vel_obs_init: obstacle initial speed (3)
# trajectory selection (1)

num_params = nx + nu + nx * (N + 1) + 1 + 3 + 3 +1
p_params = cs.SX.sym("p_params", num_params)

# === Cost Matrices ===
Qx_mat = cs.diag(cs.DM([5, 5, 30, 3, 3, 3, 8, 8])) 
Qu_mat = cs.diag(cs.DM([5, 10, 10]))
Qdu_mat = cs.diag(cs.DM([5, 12, 12]))
# Cost matrix for the final cost x_N (pcan be equal to Qx_mat or not)
Qx_terminal_mat = cs.diag(cs.DM([10, 10, 60, 6, 6, 6, 16, 16]))

# === Dynamic Parameters ===
params_dynamics = {
    'g': 9.81, 'Ax': 0.1, 'Ay': 0.1, 'Az': 0.2,
    'tau_phi': 0.5, 'tau_theta': 0.5, 'K_phi': 1.0, 'K_theta': 1.0,
    'Bx': 0.3, 'By': 0.3, 'Bz':0.6
}

# === Dynamic Functions ===
def R_rotation_matrix(phi, theta):
    return cs.vertcat(
        cs.horzcat(cs.cos(theta), 0, -cs.sin(theta)),
        cs.horzcat(cs.sin(phi)*cs.sin(theta), cs.cos(phi), cs.sin(phi)*cs.cos(theta)),
        cs.horzcat(cs.cos(phi)*cs.sin(theta), -cs.sin(phi), cs.cos(phi)*cs.cos(theta))
    )

def drone_dynamics_increment_calc(x_state, u_ctrl, pvals_dyn):
    v = x_state[3:6]
    phi = x_state[6]
    theta = x_state[7]
    T_thrust = u_ctrl[0]
    phi_ref_ctrl = u_ctrl[1]
    theta_ref_ctrl = u_ctrl[2]
    A_drag = cs.diag(cs.vertcat(pvals_dyn['Ax'], pvals_dyn['Ay'], pvals_dyn['Az']))
    dp = v
    acc = R_rotation_matrix(phi, theta) @ cs.vertcat(0, 0, T_thrust) + \
          cs.vertcat(0, 0, -pvals_dyn['g']) - A_drag @ v
    dphi = (1 / pvals_dyn['tau_phi']) * (pvals_dyn['K_phi'] * phi_ref_ctrl - phi)
    dtheta = (1 / pvals_dyn['tau_theta']) * (pvals_dyn['K_theta'] * theta_ref_ctrl - theta)
    return cs.vertcat(dp, acc, dphi, dtheta)

def step_forward_dynamics(x_state, u_ctrl, delta_t, pvals_dyn):
    return x_state + delta_t * drone_dynamics_increment_calc(x_state, u_ctrl, pvals_dyn)

def obs_dyn_linear_update(p0_obs_prev, vel_obs_const, delta_t): #linear motion
    return p0_obs_prev + vel_obs_const * delta_t

def obs_dyn_projectile_update(p0_obs_prev, vel_obs_init, delta_t, pvals_dyn):
    x = p0_obs_prev[0] + vel_obs_init[0]/pvals_dyn['Bx']*(1-mt.e**(-pvals_dyn['Bx']*delta_t))
    y = p0_obs_prev[1] + vel_obs_init[1]/pvals_dyn['By']*(1-mt.e**(-pvals_dyn['By']*delta_t))
    z = p0_obs_prev[2] + (vel_obs_init[2] + pvals_dyn['g'] / pvals_dyn['Bz']) * (1 - mt.exp(-pvals_dyn['Bz'] * delta_t)) / pvals_dyn['Bz'] - (pvals_dyn['g'] / pvals_dyn['Bz']) * delta_t
    return cs.vertcat(x, y, z)

def obs_velocity_projectile_update(v_obs_prev_step, delta_t, pvals_dyn):
    vx_new = v_obs_prev_step[0] * mt.e**(-pvals_dyn['Bx'] * delta_t)
    vy_new = v_obs_prev_step[1] * mt.e**(-pvals_dyn['By'] * delta_t)
    vz_new = (v_obs_prev_step[2] + pvals_dyn['g'] / pvals_dyn['Bz']) * mt.e**(-pvals_dyn['Bz'] * delta_t) - (pvals_dyn['g'] / pvals_dyn['Bz'])
    return cs.vertcat(vx_new, vy_new, vz_new)

# def distance_from_obs(drone_pos, sphere_pos):
#     return cs.norm_2(drone_pos - sphere_pos)

# chosing obstacle dynamic, for the moment manually
# obs_dyn_update = obs_dyn_projectile_update
obs_pos_update_fn = obs_dyn_projectile_update
obs_vel_update_fn = obs_velocity_projectile_update # for the projectile



# === Parameter extraction from p_params ===
idx = 0
x0_p = p_params[idx : idx + nx]; idx += nx
u_prev_p = p_params[idx : idx + nu]; idx += nu
x_ref_horizon_flat_p = p_params[idx : idx + nx * (N + 1)]; idx += nx * (N + 1)
x_ref_horizon_p = cs.reshape(x_ref_horizon_flat_p, (nx, N + 1)) # Refs per x_0...x_N
r_obs_p = p_params[idx]; idx += 1
p_obs_initial_p = p_params[idx : idx + 3]; idx += 3
vel_obs = p_params[idx : idx + 3]; idx+= 3 # velocity at this time instant
trajectory_type = p_params[idx]; idx += 1

distance_from_obs = cs.norm_2(x0_p[0:3] - p_obs_initial_p)
distance_threshold = 5.0

# === Cost Function building ===
# u_decision_vars contains u_0, ..., u_{N-1}
u_sequence_sim = cs.reshape(u_decision_vars, (nu, N))

x_current_sim = x0_p # Actual state in simulation. It starts with x0, ...
objective_cost = 0.0
u_hover_ref_const = cs.DM([params_dynamics['g'], 0, 0]) # hovering input ref
# penalty = 0.05 # Defining how far the UAV must stay from the obstacle
penalty_val = cs.if_else(trajectory_type == 2, 500, 10)
penalty = cs.if_else(distance_from_obs < distance_threshold, penalty_val, 0.0) 

# Costs (j = 0 to N-1)
for j in range(N):
    u_j_sim = u_sequence_sim[:, j]
    x_ref_j = x_ref_horizon_p[:, j] # Ref for state x_j

    # Cost of state x_j 
    error_x_j = x_ref_j - x_current_sim
    objective_cost += cs.mtimes([error_x_j.T, Qx_mat, error_x_j])

    # Input u_j cost
    error_u_j = u_hover_ref_const - u_j_sim
    objective_cost += cs.mtimes([error_u_j.T, Qu_mat, error_u_j])

    # Variation of input (u_j vs u_{j-1}) cost
    u_prev_for_du = u_prev_p if j == 0 else u_sequence_sim[:, j - 1]
    delta_u_j = u_j_sim - u_prev_for_du
    objective_cost += cs.mtimes([delta_u_j.T, Qdu_mat, delta_u_j])

    # Next state simulation x_{j+1}
    x_current_sim = step_forward_dynamics(x_current_sim, u_j_sim, dt, params_dynamics)

# Terminal cost for x_N (x_current_sim is now x_N at the end of the loop)
x_ref_N_terminal = x_ref_horizon_p[:, N] # Ref for the nominal state x_N
error_x_N_terminal = x_ref_N_terminal - x_current_sim
objective_cost += cs.mtimes([error_x_N_terminal.T, Qx_terminal_mat, error_x_N_terminal])


# === Penalty constraints building (f2_vector <= 0) ===
f2_penalty_constraints_list = []

# 1. Obstacle avoiding constraints
x_predict_constr = x0_p

p_obs_predict_constr = p_obs_initial_p #prediction of position

#v_obs_predict_linear = vel_obs_linear #prediction of velocity in linear case
#v_obs_predict_constr = vel_obs_init #prediction of velocity in projectile case
#v_obs_predict_constr = cs.if_else(trajectory_type== 1, vel_obs_linear, vel_obs_init) #depending on trajectory
v_obs_predict_constr = vel_obs

# Constraint for the initial state x0 w.r.t. p_obs_initial_p
r_s_val = 0.0 # Safety radius for x0 (rs=0.0m at j=0)
current_radius_inflated = r_obs_p + r_s_val
# h_sphere_j = (r_obs + r_s)^2 - ||drone_pos - obs_pos||^2. Must be<= 0.
h_sphere_term = penalty*1/(current_radius_inflated**2 - cs.sumsqr(x_predict_constr[0:3] - p_obs_predict_constr)+0.0001)

f2_penalty_constraints_list.append(h_sphere_term) 

# Constraints for x_1, ..., x_N
for j in range(N): # Loop N times for the N inputs
    u_j_constr = u_sequence_sim[:, j]
    x_predict_constr = step_forward_dynamics(x_predict_constr, u_j_constr, dt, params_dynamics) # This is x_{j+1}

    # p_obs_next_step = obs_pos_update_fn(p_obs_predict_constr, v_obs_predict_constr, dt, params_dynamics) # p_obs{j+1}

    # v_obs_next_step= obs_vel_update_fn(v_obs_predict_constr, dt, params_dynamics) # v_obs{j+1}, for projectile case

    # OBSTACLE PREDICTION UPDATE (according to the estimated trajectory)
    p_obs_next_step = cs.if_else(trajectory_type == 0, p_obs_predict_constr,
                                 cs.if_else(trajectory_type == 1, obs_dyn_linear_update(p_obs_predict_constr, v_obs_predict_constr, dt),
        obs_dyn_projectile_update(p_obs_predict_constr, v_obs_predict_constr, dt, params_dynamics))
    )

    v_obs_next_step = cs.if_else(trajectory_type == 0, v_obs_predict_constr, 
                                cs.if_else(trajectory_type == 1,
        v_obs_predict_constr, #vel remains constant if trajectory linear
        obs_velocity_projectile_update(v_obs_predict_constr, dt, params_dynamics)) #vel is updated if trajectory is projectile
    )

    p_obs_predict_constr = p_obs_next_step
    v_obs_predict_constr = v_obs_next_step


    r_s_val = 0.005 * (j + 1) # Safety radius growing for x_{j+1} 
                               # paper says 0.2m at j=N. 0.005*40 = 0.2. Correct.
    current_radius_inflated = r_obs_p + r_s_val
    h_sphere_term = penalty*1/(current_radius_inflated**2 - cs.sumsqr(x_predict_constr[0:3] - p_obs_predict_constr)+0.0001)

    f2_penalty_constraints_list.append(h_sphere_term)

# 2. Constraints on Control Input Variations
# For u_0 vs u_prev_p (phi_ref is u[1], theta_ref is u[2])
# Variation: u_curr - u_prev. Constraint: (u_curr - u_prev) - max_delta <= 0
# Constraint: (u_prev - u_curr) - max_delta <= 0 (equivalent to u_curr - u_prev >= -max_delta)
delta_phi_0 = u_sequence_sim[1, 0] - u_prev_p[1]
delta_theta_0 = u_sequence_sim[2, 0] - u_prev_p[2]
f2_penalty_constraints_list.append(delta_phi_0 - dphi_max)
f2_penalty_constraints_list.append(-delta_phi_0 - dphi_max) # equiv. delta_phi_0 >= -dphi_max
f2_penalty_constraints_list.append(delta_theta_0 - dtheta_max)
f2_penalty_constraints_list.append(-delta_theta_0 - dtheta_max)

for j in range(1, N): # Per u_j vs u_{j-1}
    delta_phi_j = u_sequence_sim[1, j] - u_sequence_sim[1, j-1]
    delta_theta_j = u_sequence_sim[2, j] - u_sequence_sim[2, j-1]
    f2_penalty_constraints_list.append(delta_phi_j - dphi_max)
    f2_penalty_constraints_list.append(-delta_phi_j - dphi_max)
    f2_penalty_constraints_list.append(delta_theta_j - dtheta_max)
    f2_penalty_constraints_list.append(-delta_theta_j - dtheta_max)

f2_penalty_vector = cs.vertcat(*f2_penalty_constraints_list)

# === Input Amplitude Constraints (Box Constraints) ===
# Applicati a u_0, ..., u_{N-1}
umin_stage_vals = [5.0, -0.35, -0.35] 
umax_stage_vals = [112, 0.35, 0.35] # rescaled on the drone weight 13.5 but never reached
umin_horizon_vals = umin_stage_vals * N
umax_horizon_vals = umax_stage_vals * N
bounds_on_u_vars = ogc.Rectangle(umin_horizon_vals, umax_horizon_vals)

# === OpEn Problem Definition ===
problem = og.builder.Problem(u_decision_vars, p_params, objective_cost) \
    .with_constraints(bounds_on_u_vars) \
    .with_penalty_constraints(f2_penalty_vector)

# === OpEn Builder Configuration ===
build_cfg = og.config.BuildConfiguration() \
    .with_build_directory("build") \
    .with_tcp_interface_config() \
    .with_build_c_bindings() # added if necessary

meta = og.config.OptimizerMeta().with_optimizer_name("drone_controller")

# penalty and tolerance may require tuning
solver_cfg = og.config.SolverConfiguration() \
    .with_tolerance(1e-4) \
    .with_initial_tolerance(1e-4) \
    .with_penalty_weight_update_factor(1.1) \
    .with_initial_penalty(1e2) \
    .with_max_outer_iterations(5)
            # to tune "how far the drone stays from the obstacle" adjust .with_penalty_weight_update_factor()
builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_cfg, solver_cfg)
# To execute the build:
builder.build()

print(f"Numero variabili di decisione (u): {u_decision_vars.shape[0]}")
print(f"Numero parametri (p): {p_params.shape[0]}")
print(f"Numero vincoli di penalità (elementi in f2_vector): {f2_penalty_vector.shape[0]}")
