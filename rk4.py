import numpy as np

def rk4(y, t , dt, f):
    k1 = np.array(f(y,t))
    k2 = np.array(f(y+ 0.5 * dt*k1,t+ 0.5*dt))
    k3 = np.array(f(y+0.5 * dt *k2,t+0.5*dt))
    k4= np.array(f(y+ dt*k3, t+dt))

    return y + dt * (k1 + 2*k2 + 2*k3 + k4)/6