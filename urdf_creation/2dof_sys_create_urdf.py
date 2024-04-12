from odio_urdf import *
import yaml
import matplotlib.pyplot as plt

import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
import time
 
from pinocchio.visualize import MeshcatVisualizer

# MUJOCO Conversion
# /media/daten/Anwendungen/mujoco-2.3.7/bin/compile /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.urdf /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.xml
# /media/daten/Anwendungen/mujoco-2.3.7/bin/simulate /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.xml
# /media/daten/Anwendungen/mujoco-2.3.7/bin/simulate /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/scene_2dof.xml

filepath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/stl_files/"

# Load the data from the yaml file
with open("./urdf_creation/param_robot.yaml") as f:
    data = yaml.safe_load(f)

def color_fun(color):
    if color == 'Grey':
        return Material(Color([0.5, 0.5, 0.5, 1]), name=color)
    elif color == 'Orange':
        return Material(Color([0.97, 0.7, 0.17, 1]), name=color)
    else:
        raise ValueError('Invalid color name')

def inertia_fun(inertia_matrix):
    ixx = inertia_matrix[0]
    iyy = inertia_matrix[1]
    izz = inertia_matrix[2]
    
    return Inertia(ixy=0, ixz=0, iyy=iyy, iyz=0, izz=izz, ixx=ixx)

def link(link_name, link_dict, color, stl_name):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    ret = Link(
        Inertial(
            Origin(link_dict['d__link']),
            Mass(value=link_dict['m']),
            inertia_fun(link_dict['d__inertiaMatrixDiag'])   # Inertia([inertia_dict(link_dict['d__inertiaMatrixDiag'])])),
            ),
        Visual(
            Origin(link_dict['d__link']),
            Geometry(Mesh(filename = filepath+stl_name, scale="0.001 0.001 0.001")),
            color_fun(color)),
        #Collision(
        #    Origin(geom_origin),from urdfpy import URDF
        #    Geometry(Mesh(filename = filepath+"collision/link_"+N+".stl")),
        #    Material(material)),
       name = link_name)
    return ret


def joint(joint_name, parent_name, child_name, link_dict, type):
    """
        Most of the joints are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    ret = Joint(
        Parent(link=parent_name),
        Child(link=child_name),
        Origin(link_dict['d__joint']),
        Axis(xyz=link_dict['axis__joint']),
        Limit(lower=link_dict['limits__joint'][0], upper=link_dict['limits__joint'][1], effort=100, velocity=10**8),
        #Safety_controller(soft_lower_limit=link_dict['limits__joint'][0],soft_upper_limit=link_dict['limits__joint'][1],
        #    k_position=0, k_velocity=10**8),
        Dynamics(damping=0, friction=0),
        type=type,
        name= joint_name)
    return ret

my_robot = Robot('2dof_sys',
    #Link(name='world'),
    link('world', data['holder'], 'Grey', 'planar_manipulator-holder_fin.stl'),
    joint('joint1', 'world', 'link1', data['holder'], "revolute"),
    link('link1', data['link1'], 'Orange', 'planar_manipulator-bulk1_fin.stl'),
    joint('joint2', 'link1', 'link2', data['link1'], "revolute"),
    link('link2', data['link2'], 'Orange', 'planar_manipulator-bulk2_fin.stl'),
    joint('jointTCP', 'link2', 'TCP', data['link2'], "fixed"),
    Link(name='TCP')
) 

print(my_robot) #Dump urdf to stdout

with open("./urdf_creation/2dof_sys.urdf", "w+") as f:
    # Write the string to the file
    str_my_robot = str(my_robot)
    mujoco_str = '''
    <mujoco>
            <compiler 
            meshdir="/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/stl_files/" 
            balanceinertia="true" 
            discardvisual="false" />
    </mujoco>\n
    '''

    str_my_robot = str_my_robot[:-9] + mujoco_str + str_my_robot[-9:]
    f.write(str_my_robot)
    f.close()

# create mujoco file
os.system('rm /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.xml')
os.system('/media/daten/Anwendungen/mujoco-2.3.7/bin/compile /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.urdf /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.xml')

# add actuator to file:
actuator_string = '''
<actuator>
    <position name="q1" joint="joint1" ctrlrange="-3.141 3.141"/>
    <position name="q2" joint="joint2" ctrlrange="-3.141 3.141"/>
</actuator>\n
'''

with open("./urdf_creation/2dof_sys.xml", "r+") as f:
    text = f.read()
    text_new= text[:-10] + actuator_string + text[-10:]
    f.seek(0)
    f.write(text_new)
    f.close()


run_mujoco=False
if run_mujoco:
    # run mujoco
    os.system('/media/daten/Anwendungen/mujoco-2.3.7/bin/simulate /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/scene_2dof.xml')

# based on https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_display_b-meshcat-viewer.html

visible_true=True
if visible_true:
    # Load the URDF model.
    # Conversion with str seems to be necessary when executing this file with ipython
    mesh_dir = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/stl_files/'
    urdf_model_path  = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys.urdf'
    #urdf_model_path  = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/pendulum.urdf'
    # https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/
    
    model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

    viz = MeshcatVisualizer(model, collision_model, visual_model)

    try:
        viz.initViewer(open=True)
    except ImportError as err:
        print(
            "Error while initializing the viewer. It seems you should install Python meshcat"
        )
        print(err)
        sys.exit(0)

    viz.loadViewerModel()

    # Display a robot configuration.
    q0 = pin.neutral(model)
    viz.display(q0)
    viz.displayVisuals(True)
    viz.display(pin.randomConfiguration(model))

    # Use forwards kinematics
    data = viz.data
    q_T = np.array([np.pi/2, np.pi/2])
    q_p_0 = np.random.randn(model.nv) * 2 # Anfangsgeschwindigkeit
    pin.forwardKinematics(model, data, q_T, q_p_0)
    frame_id = model.getFrameId('TCP')
    viz.display()
    viz.drawFrameVelocities(frame_id=frame_id)

    def tic():
        global start_time
        start_time = time.time()

    def toc():
        if 'start_time' in globals():
            elapsed_time = time.time() - start_time
            print(f"Elapsed time: {elapsed_time} seconds")
        else:
            print("Call tic() before calling toc()")

    def f(x, u):
        q = x[0:model.nq]
        q_p = x[model.nq:model.nq+model.nv]
        q_pp = pin.aba(model, data, q, q_p, u)
        return np.concatenate((q_p,q_pp))

    def runge_kutta_4(f, x, u, Ta):
        k1 = f(x, u)
        k2 = f(x + Ta/2 * k1, u)
        k3 = f(x + Ta/2 * k2, u)
        k4 = f(x + Ta * k3, u)
        x=x+Ta/6*(k1 +2*k2 +2*k3 +k4)
        return x

    # enable gravity (in -y direction)
    model.gravity.linear[:] = [0, -9.81, 0]
    
    # Simulate Dynamics
    q_0 = np.zeros(model.nq)
    q_p_0 = np.zeros(model.nv)

    Ta = 0.001
    T_sim = 10

    def sim_loop(T_sim, display_joint=False):
        tau0 = np.zeros(model.nv) # input torque

        N_samp = int(np.round(T_sim/Ta))
        q_arr = np.zeros((N_samp+1, model.nq))
        q_p_arr = np.zeros((N_samp+1, model.nv))
        q_arr[0] = q_0
        q_p_arr[0] = q_p_0
        for i in range(N_samp):
            q = q_arr[i]
            q_p = q_p_arr[i]

            # Build in Euler
            q_pp = pin.aba(model, data, q, q_p, tau0)
            q_p_next = q_p + Ta * q_pp # Euler method
            q_next = pin.integrate(model, q, Ta * q_p_next)

            #x = np.concatenate((q,q_p))
            #u = tau0

            # Custom RK 4
            #x_next = runge_kutta_4(f, x, u, Ta)

            # Custom EULER (Overhead)
            #x_p = f(x, u)
            #x_next = x + x_p*Ta

            #q_next = x_next[0:model.nq]
            #q_p_next = x_next[model.nq:model.nq+model.nv]

            # Custom Euler (no Overhead)
            #q_pp = pin.aba(model, data, q, q_p, tau0)
            #q_next = q + q_p*Ta
            #q_p_next = q_p + q_pp*Ta
            
            q_arr[i+1] = q_next
            q_p_arr[i+1] = q_p_next

            if display_joint:
                viz.display(q_next)
                viz.drawFrameVelocities(frame_id=frame_id)
        return q_arr, q_p_arr

    def my_callback(i, *args):
        viz.drawFrameVelocities(frame_id)
    
    # Show all robot methods
    #for name, function in model.__class__.__dict__.items():
    #    print(' **** %s: %s' % (name, function.__doc__))

    tic()
    q_arr, q_p_arr = sim_loop(T_sim, False)
    toc()

    plot_data = True
    if plot_data:
        t = np.linspace(0, T_sim, len(q_arr))

        plt.style.use('dark_background')
        # Create Subplots
        fig, axs = plt.subplots(2, 1, figsize=(10, 10))

        # First Subplot
        axs[0].plot(t, q_arr[:, 0], label='q_1', color='y', linewidth=1)
        axs[0].plot(t, q_arr[:, 1], label='q_2', color='b', linewidth=1)
        axs[0].set_title('Joint Configurations q over Time t')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Joint Configurations')
        axs[0].legend()
        axs[0].grid(visible=True, color='gray')
        axs[0].set_xlim([0, T_sim])

        # Second Subplot
        axs[1].plot(t, q_p_arr[:, 0], label='q_p_1', color='y', linewidth=1)
        axs[1].plot(t, q_p_arr[:, 1], label='q_p_2', color='b', linewidth=1)
        axs[1].set_title('Joint Velocities q_p over Time t')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Joint Velocities')
        axs[1].legend()
        axs[1].grid(visible=True, color='gray')
        axs[1].set_xlim([0, T_sim])

        plt.tight_layout()  # Adjust subplots to prevent overlap
        plt.show()

    while True:
        time.sleep(1.0)
        viz.play(q_arr, Ta, callback=my_callback)