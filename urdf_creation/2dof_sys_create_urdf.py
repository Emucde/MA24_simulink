from odio_urdf import *
import yaml

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
            Origin(link_dict['d__inertiaOrigin']),
            Mass(value=link_dict['m']),
            inertia_fun(link_dict['d__inertiaMatrixDiag'])   # Inertia([inertia_dict(link_dict['d__inertiaMatrixDiag'])])),
            ),
        Visual(
            Origin(link_dict['d__joint']),
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
        Origin(link_dict['d__inertiaOrigin']),
        Axis(xyz=link_dict['axis__joint']),
        Limit(lower=link_dict['limits__joint'][0], upper=link_dict['limits__joint'][1], effort=100, velocity=10**8),
        Safety_controller(soft_lower_limit=link_dict['limits__joint'][0],soft_upper_limit=link_dict['limits__joint'][1],
            k_position=0, k_velocity=10**8),
        Dynamics(damping=1),
        type=type,
        name= joint_name)
    return ret

def transmission(transmission_name, joint_name, motor_name, hw_interface='PositionJointInterface'):
    """ Definition of one transmission """
    ret = Transmission(
        transmission_name,
        Type(xmltext="transmission_interface/SimpleTransmission"),
        Transjoint(joint_name, Hardwareinterface(xmltext = hw_interface)),
        Actuator(motor_name, 
            Hardwareinterface(xmltext = hw_interface),
            Mechanicalreduction(xmltext = "1")
        )
    )
    return ret

my_robot = Robot('2dof_sys',
    #Link(name='world'),
    link('world', data['holder'], 'Grey', 'planar_manipulator-holder_fin.stl'),
    joint('joint1', 'world', 'link1', data['holder'], "revolute"),
    transmission('joint1_transmission', 'joint1', 'joint1_actuator'),
    link('link1', data['link1'], 'Orange', 'planar_manipulator-bulk1_fin.stl'),
    joint('joint2', 'link1', 'link2', data['holder'], "revolute"),
    transmission('joint2_transmission', 'joint2', 'joint2_actuator'),
    link('link2', data['link2'], 'Orange', 'planar_manipulator-bulk2_fin.stl')
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


#robot = URDF.load('./urdf_creation/2dof_sys.urdf')
#robot.show(joint1=0, joint2=0)

import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
import time
 
from pinocchio.visualize import MeshcatVisualizer
 
# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
mesh_dir = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/stl_files/'
urdf_model_path  = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/2dof_sys2.urdf'
#urdf_model_path  = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/urdf_creation/pendulum.urdf'
 
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
 
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

while True:
    time.sleep(1.0)
