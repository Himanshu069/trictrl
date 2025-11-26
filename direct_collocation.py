import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Degree of interpolating polynomial
d = 3

# Get collocation points
tau_root = np.append(0, ca.collocation_points(d, 'legendre'))

# Coefficients of the collocation equation
C = np.zeros((d+1,d+1))

# Coefficients of the continuity equation
D = np.zeros(d+1)

# Coefficients of the quadrature function
B = np.zeros(d+1)

# Construct polynomial basis
for j in range(d+1):
    # Construct Lagrange polynomials to get the polynomial basis at the collocation point
    p = np.poly1d([1])
    for r in range(d+1):
        if r != j:
            p *= np.poly1d([1, -tau_root[r]]) / (tau_root[j]-tau_root[r])

    # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
    D[j] = p(1.0)

    # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
    pder = np.polyder(p)
    for r in range(d+1):
        C[j,r] = pder(tau_root[r])

    # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
    pint = np.polyint(p)
    B[j] = pint(1.0)

# Time horizon
T = 10.0

# Declare model variables
theta = ca.SX.sym('theta')
thetadot = ca.SX.sym('thetadot')
phi = ca.SX.sym('phi')
phidot = ca.SX.sym('phidot')
x = ca.vertcat(theta, thetadot, phi, phidot)
u = ca.SX.sym('u')

M = 0.01825     #mblb+mwlw
mb = 0.286
mw = 0.079
Iw = 0.000040394
Ib = 0.0004370975
lb = 0.05
lw = 0.05
g_grav = 9.81
I = Ib + mb*lb*lb + mw*lw*lw

# Model equations
sin_theta = ca.sin(theta)
cos_theta = ca.cos(theta)
theta_dot_dot = (-u+M*g_grav*sin_theta)/I
phi_dot_dot = u*(I+Iw)/(I*Iw) - M*g_grav*sin_theta/I

xdot = ca.vertcat(thetadot, theta_dot_dot,phidot, phi_dot_dot)

E_KIN = 0.5 * I * thetadot**2 + 0.5 * Iw *(thetadot+phidot)**2
E_POT = -M *g_grav* cos_theta

# Objective term    
# Q_theta = 100.0
# Q_thetadot = 0.0
# Q_phidot = 0.0
# L = Q_theta*theta**2 + Q_thetadot*thetadot**2 +Q_phidot*phidot**2 
L = E_KIN - E_POT
# Continuous time dynamics
f = ca.Function('f', [x, u], [xdot, L], ['x', 'u'], ['xdot', 'L'])

# Control discretization
N = 50 # number of control intervals
h = T/N

# Start with an empty NLP
w=[]
w0 = []
lbw = []
ubw = []
J = 0
g=[]
lbg = []
ubg = []

# For plotting x and u given w
x_plot = []
u_plot = []

# "Lift" initial conditions
Xk = ca.MX.sym('X0', 4)
w.append(Xk)
lbw.append([0.7455,0,0,0 ])
ubw.append([0.7455,0,0,0])
w0.append([0.7455,0,0,0])
x_plot.append(Xk)

# Formulate the NLP
for k in range(N):
    # New NLP variable for the control
    Uk = ca.MX.sym('U_' + str(k))
    w.append(Uk)
    lbw.append([-0.5])
    ubw.append([0.5])
    progress_u = (k + 0.5) / N
    theta_guess_u = 0.7455 * (1 - 3 * progress_u**2 + 2 * progress_u**3)
    thetaddot_guess = 0.7455 / (T**2) * (-6 + 12 * progress_u)
    sin_theta_u = np.sin(theta_guess_u)
    u_guess = M * g_grav * sin_theta_u - I * thetaddot_guess
    w0.append([u_guess])
    u_plot.append(Uk)

    # State at collocation points
    Xc = []
    for j in range(d):
        Xkj = ca.MX.sym('X_'+str(k)+'_'+str(j), 4)
        Xc.append(Xkj)
        w.append(Xkj)
        lbw.append([-0.7455, -20.0, -np.pi, -100.0])
        ubw.append([0.7455,  20.0, +np.pi,100.0])
        progress = (k + tau_root[j+1]) / N
        theta_guess = 0.7455 * (1 - 3 * progress**2 + 2 * progress**3)
        thetadot_guess = 0.7455 * (-6 * progress + 6 * progress**2) / T
        w0.append([theta_guess, thetadot_guess, 0.0, 0.0])      

    # Loop over collocation points
    Xk_end = D[0]*Xk
    for j in range(1,d+1):
       # Expression for the state derivative at the collocation point
       xp = C[0,j]*Xk
       for r in range(d):
           xp = xp + C[r+1,j]*Xc[r]

       # Append collocation equations
       fj, qj = f(Xc[j-1],Uk)
       g.append(h*fj - xp)
       lbg.append([0, 0,0,0])
       ubg.append([0, 0,0,0])

    #    g.append( Xc[j-1][3] - Xc[j-2][3] - h * (Uk/Iw - Xc[j-1][1]) )       
    #    lbg.append([0])
    #    ubg.append([0])

       # Add contribution to the end state
       Xk_end = Xk_end + D[j]*Xc[j-1]

       # Add contribution to quadrature function
       J = J + B[j]*qj*h

    # New NLP variable for state at end of interval
    Xk = ca.MX.sym('X_' + str(k+1), 4)
    w.append(Xk)
    if k == N-1:  # Final state
        lbw.append([-0.05, -5.0, -np.pi, -100.0])  # theta=0, thetadot=0, phi/phidot free
        ubw.append([0.05, 5.0, np.pi, 100.0])
        w0.append([0.0, 0.0, 0.0, 0.0])
    else:  # Intermediate states
        lbw.append([-0.7455, -20.0, -np.pi, -100.0])
        ubw.append([0.7455, 20.0, np.pi, 100.0])
        progress = (k + 1) / N  # 0 to 1 over trajectory
        theta_guess = 0.7455 * (1 - 3 * progress**2 + 2 * progress**3)
        thetadot_guess = 0.7455 * (-6 * progress + 6 * progress**2) / T
        w0.append([theta_guess, thetadot_guess, 0.0, 0.0])    
    
    x_plot.append(Xk)

    # Add equality constraint
    g.append(Xk_end-Xk)
    lbg.append([0, 0, 0, 0])
    ubg.append([0, 0, 0, 0])

# Concatenate vectors
w = ca.vertcat(*w)
g = ca.vertcat(*g)
x_plot = ca.horzcat(*x_plot)
u_plot = ca.horzcat(*u_plot)
w0 = np.concatenate(w0)
lbw = np.concatenate(lbw)
ubw = np.concatenate(ubw)
lbg = np.concatenate(lbg)
ubg = np.concatenate(ubg)

# Create an NLP solver
prob = {'f': J, 'x': w, 'g': g}
solver = ca.nlpsol('solver', 'ipopt', prob)

# Function to get x and u trajectories from w
trajectories = ca.Function('trajectories', [w], [x_plot, u_plot], ['w'], ['x', 'u'])



# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
x_opt, u_opt = trajectories(sol['x'])
x_opt = x_opt.full() # to numpy array
u_opt = u_opt.full() # to numpy array

print("Max control: ", np.max(np.abs(u_opt)))
# Plot the result
tgrid = np.linspace(0, T, N+1)
plt.figure(1)
plt.clf()
plt.plot(tgrid, x_opt[0], '-')
plt.plot(tgrid, x_opt[1], '-')
# plt.step(tgrid, np.append(np.nan, u_opt[0]), '-.')
plt.xlabel('t')
plt.legend(['theta','thetadot','u'])
plt.grid()
plt.show()

plt.figure(2)
plt.clf()
plt.plot(tgrid, x_opt[3], '-')
# plt.step(tgrid, np.append(np.nan, u_opt[0]), '-.')
plt.xlabel('t')
plt.legend(['phidot','u'])
plt.grid()
plt.show()

plt.figure(3)
plt.clf()
plt.step(tgrid, np.append(np.nan, u_opt[0]), '-.')
plt.xlabel('u')
plt.legend(['Control input(u)'])
plt.grid()
plt.show()

