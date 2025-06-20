from triangle_dynamics import f
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def reduced_dynamics(theta, theta_dot, phi=0.0, phi_dot=0.0):
    y = [theta, theta_dot, phi, phi_dot]
    dy = f(y, 0) #since this is an autonomous system we can set t=0
    return dy[0], dy[1], dy[2], dy[3]

t_start = 0.0
t_end = 10.0
dt = 0.01
num_steps = int((t_end - t_start) / dt)
time = np.arange(t_start, t_end, dt)

theta_vals = np.linspace(-np.pi, np.pi, 20)
theta_dot_vals = np.linspace(-10, 10, 20)
phi_vals = np.linspace(-np.pi, np.pi, 20)
phi_dot_vals = np.linspace(-500,500,50)

THETA, THETA_DOT = np.meshgrid(theta_vals, theta_dot_vals)
DTHETA = np.zeros_like(THETA)
DTHETA_DOT = np.zeros_like(THETA_DOT)

for i in range(THETA.shape[0]):
    for j in range(THETA_DOT.shape[1]):
        dtheta, dtheta_dot,_,_ = reduced_dynamics(THETA[i, j], THETA_DOT[i, j])
        DTHETA[i, j] = dtheta
        DTHETA_DOT[i, j] = dtheta_dot

#1 Phase Vector Field: θ vs θ̇
plt.figure(figsize=(8, 6))
plt.quiver(THETA, THETA_DOT, DTHETA, DTHETA_DOT, color='blue', angles='xy')
plt.xlabel('θ')
plt.ylabel(r'$\dot{\theta}$')
plt.title(r'Phase Vector Field: θ vs $\dot{\theta}$')
plt.grid(True)
plt.savefig('results/unforced_system_phase1.png')
plt.show()


#2 Plot θ vs φ (Pendulum Angle vs Wheel Angle)
fig2 = plt.figure(figsize=(8, 6))
THETA, PHI = np.meshgrid(theta_vals, phi_vals)
DTHETA = np.zeros_like(THETA)
DPHI = np.zeros_like(PHI)
# print("THETA", THETA)
# print("PHI", PHI)
for i in range(THETA.shape[0]):
    for j in range(PHI.shape[1]):
        dtheta, dtheta_dot, dphi, dphi_dot = reduced_dynamics(THETA[i,j],0.1, PHI[i,j],0.1)
        DTHETA[i,j] = dtheta
        DPHI[i,j] = dphi

plt.quiver(THETA, PHI, DTHETA, DPHI, color='blue', angles='xy')
plt.xlabel('θ̇')
plt.ylabel('φ')
plt.title('2D Plot: θ vs φ')
plt.grid(True)
plt.savefig('results/unforced_system_phase2.png')
plt.show()

#3 Plot θ̇ vs φ̇ (Angular Velocity vs Wheel Angle)
fig2 = plt.figure(figsize=(8, 6))
THETA_DOT, PHI_DOT = np.meshgrid(theta_dot_vals, phi_dot_vals)
DTHETA_DOT = np.zeros_like(THETA_DOT)
DPHI_DOT = np.zeros_like(PHI_DOT)

for i in range(THETA_DOT.shape[0]):
    for j in range(PHI_DOT.shape[1]):
        dtheta, dtheta_dot, dphi, dphi_dot = reduced_dynamics(0.1,THETA_DOT[i,j],0.1, PHI_DOT[i,j])
        DTHETA_DOT[i,j] = dtheta_dot
        DPHI_DOT[i,j] = dphi_dot

plt.quiver(THETA_DOT, PHI_DOT, DTHETA_DOT, DPHI_DOT, color='blue', angles='xy')
plt.xlabel(r'$\dot{\theta}$(Angular Velocity of Pendulum)')
plt.ylabel('φ̇ (Angular Velocity of Wheel)')
plt.title(r'2D Plot: $\dot{\theta}$ vs φ̇')
plt.grid(True)
plt.savefig('results/unforced_system_phase3.png')
plt.show()
