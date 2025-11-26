import numpy as np
from rk4 import rk4
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon,Circle
from matplotlib.transforms import Affine2D
from matplotlib import transforms
import math
from nn_inference import nmpc_forward

dt = 0.01
T = 5.0
t_vals = np.arange(0, T , dt)

def rk4(y, t , dt, f,u):
    k1 = np.array(f(y,t,u))
    k2 = np.array(f(y+ 0.5 * dt*k1,t+ 0.5*dt,u))
    k3 = np.array(f(y+0.5 * dt *k2,t+0.5*dt,u))
    k4= np.array(f(y+ dt*k3, t+dt,u))

    return y + dt * (k1 + 2*k2 + 2*k3 + k4)/6

def f(y,t,u):    
    
    theta, theta_dot, phi, phi_dot = y

    mb = 0.286
    mw = 0.079
    M = 0.01825     #mblb+mwlw
    Iw = 0.000040394
    Ib = 0.0004370975
    lb = 0.05
    lw = 0.05
    g = 9.81
    I = Ib + mb*lb*lb + mw*lw*lw    #moment of inertia of the wheel

    theta_dot_dot = (-u+M*g*np.sin(theta))/I
    phi_dot_dot = (u)/(Iw) - theta_dot_dot
    
    return [theta_dot, theta_dot_dot, phi_dot, phi_dot_dot]

def simulate(y0):
    y = y0
    traj = [y.copy()]
    u_traj = []
    u_traj.append(0.0)
    for t in (t_vals[:-1]):
        u = nmpc_forward([y[0],y[1],y[3]])  # control output (array)
        u = float(u)               # make scalar
        u_traj.append(u) 
        y = rk4(y, t, dt, f, u)
        traj.append(y.copy())

    return np.array(traj), np.array(u_traj)

n_trials = 1000
trajectories = []
controls = []

theta_vals = np.linspace(-np.pi, np.pi, n_trials)       
theta_dot_vals = np.linspace(-50.0, 50.0, n_trials)      
phi_dot_vals = np.linspace(-100.0, 100.0, n_trials) 

for i in range(n_trials):
    theta0 = theta_vals[i]
    theta_dot0 = theta_dot_vals[i]
    phi0 = 0.0
    phi_dot0 = phi_dot_vals[i]

    y0 = [theta0, theta_dot0, phi0, phi_dot0]
    traj, u_traj = simulate(y0)
    trajectories.append(traj)
    controls.append(u_traj)

# Save all trajectories for later analysis
np.savez("controller_trajectories.npz", trajectories=trajectories, t_vals=t_vals)


# -------------------- Visualization --------------------
plt.figure(figsize=(12, 10))

# θ (angle)
plt.subplot(4, 1, 1)
for traj in trajectories:
    plt.plot(traj[:, 0], color='blue',alpha=0.15)  # solid blue line
plt.title("Neural NMPC Controller - 100 Trajectories")
plt.ylabel("θ [rad]")
plt.grid(True)

# θ̇ (angular velocity)
plt.subplot(4, 1, 2)
for traj in trajectories:
    plt.plot(traj[:, 1], color='blue',alpha=0.15)
plt.ylabel("θ̇ [rad/s]")
plt.grid(True)

# φ̇ (wheel angular velocity)
plt.subplot(4, 1, 3)
for traj in trajectories:
    plt.plot(traj[:, 3], color='blue',alpha=0.15)
plt.ylabel("φ̇ [rad/s]")
plt.xlabel("Time [s]")
plt.grid(True)

plt.subplot(4, 1, 4)
for u_traj in controls:
    plt.step(t_vals, u_traj, where='post', color='red',alpha=0.15, linewidth=0.5)
    # plt.plot(u_traj, color='red', linewidth=1)  # red for control
plt.ylabel("u [Nm]")
plt.xlabel("Time steps")
plt.grid(True)

plt.tight_layout()
plt.show()

traj_idx = 1  # for example, the 6th trajectory

traj = trajectories[traj_idx]
u_traj = controls[traj_idx]
print("traj last value", traj[-10:])
print("u_traj last value", u_traj[-10:])
# Create time vector for this trajectory
# t_traj = np.arange(0, dt*len(traj), dt)  

plt.figure(figsize=(12, 10))

# θ (angle)
plt.subplot(4, 1, 1)
plt.plot(traj[:, 0], color='blue',alpha=0.15, linewidth=2)
plt.ylabel("θ [rad]")
plt.grid(True)

# θ̇ (angular velocity)
plt.subplot(4, 1, 2)
plt.plot(traj[:, 1], color='blue',alpha=0.15, linewidth=2)
plt.ylabel("θ̇ [rad/s]")
plt.grid(True)

# φ̇ (wheel angular velocity)
plt.subplot(4, 1, 3)
plt.plot(traj[:, 3], color='blue',alpha=0.15, linewidth=2)
plt.ylabel("φ̇ [rad/s]")
plt.grid(True)

# Control input u
plt.subplot(4, 1, 4)
plt.step(t_vals , u_traj, where='post', color='red',alpha=0.15, linewidth=2)
plt.ylabel("u [Nm]")
# plt.xlabel("Time [s]")
plt.yticks(np.arange(-0.5, 0.6, 0.1))
plt.grid(True)
plt.tight_layout()
plt.show()