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

plot_true=True
if plot_true:
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

    q1 = np.array([np.pi/2, np.pi/2])
    q1 = q0
    v0 = np.random.randn(model.nv) * 2 # Anfangsgeschwindigkeit
    v0 = v0*0

    data = viz.data
    pin.forwardKinematics(model, data, q1, v0)

    frame_id = model.getFrameId('TCP')
    viz.display()

    viz.drawFrameVelocities(frame_id=frame_id)

    model.gravity.linear[:] = [0, -9.81, 0]
    dt = 0.001

    def sim_loop():
        tau0 = np.zeros(model.nv)#-0.001 # input torque
        qs = [q1]
        vs = [v0]
        nsteps = 10000
        for i in range(nsteps):
            q = qs[i]
            v = vs[i]
            a1 = pin.aba(model, data, q, v, tau0)
            vnext = v + dt * a1
            qnext = pin.integrate(model, q, dt * vnext)
            qs.append(qnext)
            vs.append(vnext)
            viz.display(qnext)
            viz.drawFrameVelocities(frame_id=frame_id)
        return qs, vs
 
    qs, vs = sim_loop()

    def my_callback(i, *args):
        viz.drawFrameVelocities(frame_id)
    
    # Show all robot methods
    #for name, function in model.__class__.__dict__.items():
    #    print(' **** %s: %s' % (name, function.__doc__))

    plot_data = False
    if plot_data:
        qs_arr = np.array(qs)
        ts = np.linspace(0, 10000*dt, 10000 + 1)

        plt.style.use('dark_background')
        plt.figure(figsize=(10, 5))

        # Plot the joint configurations qs over time ts
        plt.plot(ts, qs_arr[:, 0], label='q1', color='y', linewidth=1)
        plt.plot(ts, qs_arr[:, 1], label='q2', color='b', linewidth=1)

        # Set the title and labels
        plt.title('Joint Configurations qs over Time ts')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Configurations')

        # Show the legend
        plt.legend()

        # Show the plot
        plt.show()

    while True:
        time.sleep(1.0)
        viz.play(qs, dt, callback=my_callback)