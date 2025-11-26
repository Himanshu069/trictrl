import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import math
import nn_inference
import matplotlib.pyplot as plt

xml_path = 'mujoco/triangle.xml' #xml file (assumes this is in the same folder as this file)
simend = 100 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

control_dt = 0.02  # 50 Hz NMPC
control_timer = 0.0
u_current = 0.0
time_log = []
phi_log=[]
theta_log = []
theta_dot_log = []
u_log = []

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    data.ctrl[:] = 0.0

max_control = 0.0
integral_error = 0.0 
def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    # desired_triangle_angle = 0
    # kp, kd = 0.25, 0.1
    # kw = 0.09
    global control_timer, u_current
    # global integral_error
    # global max_control
    controller_start_delay = 2.0
    if data.time < controller_start_delay:
        data.ctrl[:] = 0.0
        return


    if data.time - control_timer >= 0.04:  # 25 Hz
        control_timer = data.time
        x_input = np.array([[data.qpos[0], data.qvel[0]]])
        u_current = float(nn_inference.nmpc_forward(x_input))
        # u_current = np.clip(u_current, -0.5, 0.5)
        print(f"t={data.time:.2f}, θ={data.qpos[0]:.3f}, u={u_current:.5f}")

    data.ctrl[0] = -u_current 

    # if abs(data.qpos[0])<0.2:
    #     data.ctrl[0] = -u_current + 0.039*data.qvel[0]
    # else:
    # data.ctrl[0] = -u_current 
    # x_current = np.array([data.qpos[0], data.qvel[0], data.qpos[1], data.qvel[1]])
    # u_current = get_policy_action(x_current)
    # print("control input", u_current)
    # data.ctrl[0] = u_current

    # u= kp*(data.qpos[0]-0) + kd*data.qvel[0] 
    # u = np.clip(u, -0.25, 0.25)
    # data.ctrl[0] = u
    # print("control input",data.ctrl[0])
    # if data.ctrl[0]>max_control:
    #     max_control = data.ctrl[0]
    # print("max torque is", max_control)
    # # # # - kw * data.qvel[1]
    # # if math.fabs(data.qpos[0])< 0.087 :
    # u_balance= kp*(data.qpos[0]-0) + kd*data.qvel[0] 

    # kp_vel, ki_vel = 5.0, 1.0
    # dt = model.opt.timestep
    # print("dt ", dt)
    # vel_error = 0.0 - data.qvel[1]
    # integral_error += vel_error * dt
    # integral_error = np.clip(integral_error, -0.1, 0.1)
    # u_velocity = kp_vel * vel_error + ki_vel * integral_error
    # print("u_balance ", u_balance)
    # print("u_velocity ", u_velocity)

    # if math.fabs(data.qpos[0])< 0.09:   
    #     data.ctrl[0] = u_balance + u_velocity
    # else:
    #     data.ctrl[0] = u_balance
    #     integral_error = 0.0
    # #     return

    
    # M = 0.01825     #mblb+mwlw
    # mb = 0.286
    # mw = 0.079
    # Iw = 0.000040394
    # Ib = 0.0004370975
    # lb = 0.05
    # lw = 0.05
    # g = 9.81
    # I = Ib + mb*lb*lb + mw*lw*lw
    # E = 0.5 * I * math.pow(data.qvel[0],2) + 0.5 * Iw * math.pow((data.qvel[0]+data.qvel[1]),2) -M *g* math.cos(data.qpos[0])
    # Eref = - M * g
    # # kv = 200
    # # kE = 0.0001
    # # kp = 0.01
    # # kd = 0.05
    # # detD = (mb*lb*lb + mw*lw*lw +Ib + Iw)*Iw - Iw*Iw
    # # k1 = (kv * Iw * M *g)/(detD)
    # # k2 = (kv * (mb*lb*lb + mw*lw*lw +Ib + Iw))/detD
    # ke=400.0
    # kv=4.0
    # ku=0.0002
    # k=1.0
    # data.ctrl[0] = ku *(kv * data.qvel[1]-ke*k*(E-Eref)*data.qvel[0])

    # # torque = (-kd*data.qvel[1]-kp*data.qpos[1]+k1*math.sin(data.qpos[0]))/(kE*E + k2)
    # # data.ctrl[0] = torque
    # ku=0.1
    # kv=10
    # ke=1000
    # k=1
    # data.ctrl[0] = - ku *(-kv* data.qvel[1] +ke*k*(E-Eref)*data.qvel[0])


    # else:
    
    
def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.azimuth = -91.79999999999993 ; cam.elevation = -39.600000000000016 ; cam.distance =  0.7590817024851124
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

data.qpos[0] = -0.74
# data.qvel[0] = 0
# data.qvel[1] = 0

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    if data.time>0.0:
        time_log.append(data.time)
        theta_log.append(data.qpos[0])  
        phi_log.append(data.qvel[1])   # pendulum angle
        theta_dot_log.append(data.qvel[0]) # angular velocity
        u_log.append(u_current) 
    # #x,y,z position of free joint
    # print(data.qpos[2])
    # print(data.qpos[2])
    # print("qpos size:", len(data.qpos))
    print("pendulum angle", data.qpos[0])
    print("Wheel velocity:", data.qvel[1]) 
    if data.qpos[1]>6.28:
        angle = data.qpos[1] - math.floor(data.qpos[1]/6.28)*6.28
    else:
        angle = data.qpos[1]

    print("Wheel joint angle change:", angle) 
    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()


plt.figure(figsize=(10,6))
plt.subplot(4,1,1)
plt.plot(time_log, theta_log)
plt.ylabel("Pendulum Angle ")

plt.subplot(4,1,2)
plt.plot(time_log, theta_dot_log)
plt.ylabel("Pendulum Velocity ")

plt.subplot(4,1,3)
plt.step(time_log, phi_log, where='post')
plt.ylabel("Wheel velocity")
plt.xlabel("Time (s)")

plt.subplot(4,1,4)
plt.step(time_log, u_log, where='post')
plt.ylabel("Control Input (u)")
plt.xlabel("Time (s)")

plt.tight_layout()
plt.show()