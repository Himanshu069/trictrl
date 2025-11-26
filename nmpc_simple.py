import casadi as ca
import numpy as np
import math

M = 0.01825     #mblb+mwlw
mb = 0.286
mw = 0.079
Iw = 0.000040394
Ib = 0.0004370975
lb = 0.05
lw = 0.05
g_grav = 9.81
I = Ib + mb*lb*lb + mw*lw*lw
    # E = 0.5 * I * math.pow(data.qvel[0],2) + 0.5 * Iw * math.pow((data.qvel[0]+data.qvel[1]),2) -M *g* math.cos(data.qpos[0])
# Eref = - M * g_grav
dt = 0.05
N = 60

# def wrap_angle_casadi(theta):
#     return theta - 2*ca.pi*ca.floor((theta + ca.pi)/(2*ca.pi))

theta = ca.SX.sym('theta')
# theta = wrap_angle_casadi(theta_unwrapped)
thetadot = ca.SX.sym('thetadot')
phi = ca.SX.sym('phi')
phidot = ca.SX.sym('phidot')
states = ca.vertcat(theta, thetadot, phi, phidot)
n_states = states.size1()  # should be 4

u = ca.SX.sym('u')
n_controls = u.size1()     

# Decision variables for the entire horizon
# X will be of size (n_states x (N+1)) and U of size (n_controls x N)
X = ca.SX.sym('X', n_states, N+1)
U = ca.SX.sym('U', n_controls, N)


def rwp(state,u):
    theta, thetadot , phi, phidot = state[0], state[1], state[2], state[3]
    sin_theta = ca.sin(theta)
    
    theta_dot_dot = (-u-M*g_grav*sin_theta)/I
    phi_dot_dot = u/Iw - theta_dot_dot

    return ca.vertcat(thetadot,theta_dot_dot,phidot,phi_dot_dot)



def rk4_step(state, u, dt, dynamics):
    """
    state: current state (CasADi SX or MX)
    u: control input
    dt: time step
    dynamics: function(state, u) -> state_dot
    """
    k1 = dynamics(state, u)
    k2 = dynamics(state + dt/2 * k1, u)
    k3 = dynamics(state + dt/2 * k2, u)
    k4 = dynamics(state + dt * k3, u)
    
    next_state = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    return next_state

f = lambda state, u: rk4_step(state, u, dt, rwp)


obj = 0 #accumulated cost
g = [] #equality constraints

X0 = ca.SX.sym('X0', n_states)

g.append(X[:, 0] - X0)

Q_theta = 1000.0     # pendulum angle weight
Q_thetadot = 100.0     # pendulum angular velocity weight

# Loop over horizon
for k in range(N):
    th = X[0, k]       # pendulum angle
    th_dot = X[1, k]   # pendulum angular velocity

    # Stage cost: only pendulum angle and velocity
    stage_cost = Q_theta * (th-3.14159)**2 + Q_thetadot * th_dot**2
    obj += stage_cost

    # Dynamics constraints as before
    x_next = f(X[:, k], U[:, k])
    g.append(X[:, k+1] - x_next)

# Terminal cost
thN = X[0, N]
thdotN = X[1, N]
obj += Q_theta *(thN-3.14159)**2 + Q_thetadot * thdotN**2

theta_min = 2.396
theta_max = 3.8868

thetadot_min = -10.0
thetadot_max = 10.0

u_min = -0.5
u_max = 0.5

phidot_min = -100.0
phidot_max = 100.0

opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
n_decision = int(opt_vars.numel())

lbx = -np.inf * np.ones(n_decision)
ubx =  np.inf * np.ones(n_decision)

for i in range(N+1):
    base = i * n_states
    # theta
    lbx[base + 0] = theta_min
    ubx[base + 0] = theta_max
    # thetadot
    lbx[base + 1] = thetadot_min
    ubx[base + 1] = thetadot_max
    # phi (no explicit bound provided -> keep +/-inf)
    # phidot
    lbx[base + 3] = phidot_min
    ubx[base + 3] = phidot_max

start_u = n_states * (N+1)
for i in range(N):
    idx = start_u + i * n_controls  # index for control at time step i
    lbx[idx] = u_min
    ubx[idx] = u_max

# opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
g_concat = ca.vertcat(*g)

g_bound = np.zeros(int(g_concat.numel()))
lbg = g_bound
ubg = g_bound

nlp_problem = {
    'f': obj,       # Objective function
    'x': opt_vars,  # Decision variables
    'g': g_concat,  # Constraints
    'p': X0         # Parameter: initial state
}

opts = {'ipopt.print_level': 0, 'print_time': 0}
solver = ca.nlpsol('solver', 'ipopt', nlp_problem, opts)

init_guess = np.zeros(int(opt_vars.numel()))
x_current = np.array([3.886, 0.0, 0.0, 0.0])

# sol = solver(x0=init_guess, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=x_current)

# opt_solution = sol['x'].full().flatten()
# X_opt = opt_solution[: (N+1)*n_states ].reshape((n_states, N+1))
# U_opt = opt_solution[(N+1)*n_states : ].reshape((n_controls, N))

# print("X_opt shape:", X_opt.shape)
# print("U_opt shape:", U_opt.shape)

X_init = np.tile(x_current.reshape(-1,1), (1, N+1))
U_init = np.zeros((n_controls, N))
prev_sol = np.concatenate([X_init.flatten(), U_init.flatten()])

def get_nmpc_control(x_current):
    global prev_sol
    sol = solver(x0=prev_sol, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=x_current)
    sol_opt = sol['x'].full().flatten()
    
    prev_sol = shift_prev_solution(sol_opt)  # warm-start for next iteration
    
    u_start_idx = (N+1) * n_states
    return sol_opt[u_start_idx]

def shift_prev_solution(sol_opt):
    """Shift the previous optimal trajectory forward by one step."""
    X_flat = sol_opt[: (N+1)*n_states]
    U_flat = sol_opt[(N+1)*n_states :]

    X_mat = X_flat.reshape((n_states, N+1))
    U_mat = U_flat.reshape((n_controls, N))

    # Shift X and U forward
    X_shift = np.hstack((X_mat[:, 1:], X_mat[:, -1:]))  # repeat last column
    U_shift = np.hstack((U_mat[:, 1:], U_mat[:, -1:]))  # repeat last control

    # Flatten again for solver input
    shifted = np.concatenate([X_shift.flatten(), U_shift.flatten()])
    return shifted

