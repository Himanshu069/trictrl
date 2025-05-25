import numpy as np
from rk4 import rk4
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon,Circle
from matplotlib.transforms import Affine2D
from matplotlib import transforms
import math

def f(y,t):    
    
    theta, theta_dot, phi, phi_dot = y
    tm = 0 #torque motor
    tf = 0 #torque friction
    g = 9.81 #gravity
    M = 2.0 #combined mass
    I = 1.0 #Combined moment of inertia
    Iw = 0.1 #moment of inertia of the wheel
    
    theta_dot_dot = (-tm+tf+M*g*np.sin(theta))/I
    phi_dot_dot = (tm+tf)*(I+Iw)/(I*Iw) - (M*g*np.sin(theta))/(I)
    
    return [theta_dot, theta_dot_dot, phi_dot, phi_dot_dot]

def simulation(y_initial,f):
    
    t_start = 0.0
    t_end = 10.0
    dt = 0.01
    num_steps = int((t_end-t_start)/dt)
    time = np.arange(t_start, t_end, dt)
    
    theta_values=[]
    theta_dot_values=[]
    phi_values=[]
    phi_dot_values=[]
    y = y_initial
    
    for t in time:
        theta_values.append(y[0])
        theta_dot_values.append(y[1])
        phi_values.append(y[2])
        phi_dot_values.append(y[3])
        y= rk4(y,t,dt,f)
    
    ans = np.array([theta_values, theta_dot_values, phi_values, phi_dot_values])
    return ans
    
def visualization(y_initial):
    
    side_length = 2  
    height = (np.sqrt(3) / 2) * side_length 
    
    angles,_,phi_values,_ = simulation(y_initial,f)
    
    x1 = -side_length * np.cos(math.pi / 3)
    y1 = side_length * np.sin(math.pi / 3)
    x2 = side_length * np.cos(math.pi / 3)
    y2 = side_length * np.sin(math.pi / 3)

    triangle = np.array([
        [0, 0],  # Pivot
        [x1, y1],  
        [x2, y2]  
    ])

    # **Circle Offset from Centroid of Triangle**
    centroid_x = (0 + x1 + x2) / 3
    centroid_y = (0 + y1 + y2) / 3
    wheel_center_x = centroid_x   # Adjust offset
    wheel_center_y = centroid_y + 0.1  # Adjust offset
    circle_radius = 0.3  # Radius of the small rotating circle


    stop_angle = np.arctan2(height, side_length / 2)  # Maximum rotation

    fig, ax = plt.subplots()
    plt.axhline(y=0, color='black', linewidth=2)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.axis('off') 
    
    polygon = Polygon(triangle, closed=True, color='blue', edgecolor='black')
    ax.add_patch(polygon)

    circle = Circle((wheel_center_x, wheel_center_y), circle_radius, color='red', fill=True, linewidth=2)
    ax.add_patch(circle)
    
    dash_x = [wheel_center_x, wheel_center_x + circle_radius]
    dash_y = [wheel_center_y, wheel_center_y]
    dash_line, = ax.plot(dash_x, dash_y, color='black', linewidth=2)
    
    def update(frame):
        theta = angles[frame]
        phi = phi_values[frame]
        
        if theta>=stop_angle or theta<=-stop_angle:
            ani.event_source.stop()
            writer.finish()
        
        polygon.set_transform(
            transforms.Affine2D()
            .rotate_deg_around(0  , 0, np.degrees(theta))
            + ax.transData
        )
        
        # circle.set_transform(
        # transforms.Affine2D()
        # .rotate_deg_around(0, 0, np.degrees(theta)) + ax.transData
        # )
        
        new_circle_x = wheel_center_x * np.cos(theta) - wheel_center_y * np.sin(theta)
        new_circle_y = wheel_center_x * np.sin(theta) + wheel_center_y * np.cos(theta)
        circle.center = (new_circle_x, new_circle_y)
        
        dash_x_new = [new_circle_x, new_circle_x + circle_radius * np.cos(theta + phi)]
        dash_y_new = [new_circle_y, new_circle_y + circle_radius * np.sin(theta + phi)]
        dash_line.set_data(dash_x_new, dash_y_new)
        
        return polygon,circle,dash_line
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=25, metadata=dict(artist='Himanshu'), bitrate=1800)
    
    ani = animation.FuncAnimation(fig, update, frames=len(angles), interval=40, blit=True)
    
    ani.save('triangle.mp4', writer=writer)

    plt.show()
    
if __name__ == "__main__":
    theta = -math.pi/6
    theta_dot = 3
    phi = 0.0
    phi_dot = 50
    y_initial = [theta, theta_dot, phi, phi_dot]
    visualization(y_initial)