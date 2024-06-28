import time
import unittest
import os
import numpy as np
import casadi as cs
import pinocchio as pin
from pinocchio import casadi as cpin

def SX00_to_SX0(J_fun, q, q_p=None):
    if q_p is None:
        J_val = J_fun(q)
    else:
        J_val = J_fun(q, q_p)

    for row in range(J_val.shape[0]):
        for col in range(J_val.shape[1]):
            try:
                if(J_val[row, col] == 0):
                    J_val[row, col] = 0
            except:
                pass
    if q_p is None:
        return cs.Function(J_fun.name(), [q], [J_val], J_fun.name_in(), J_fun.name_out())
    else:
        return cs.Function(J_fun.name(), [q, q_p], [J_val], J_fun.name_in(), J_fun.name_out())

robot_name = "fr3_7dof"
# robot_name = "fr3_6dof"
# robot_name = "ur5e"

if "fr3" in robot_name:
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_FR3')
    root_link = "fr3_link0"
    end_link = "fr3_hand_tcp"
    if "7dof" in robot_name:
        urdf_path = os.path.join(os.path.dirname(__file__), 'fr3_7dof.urdf')
        s_functions_path = './s_functions/fr3_7dof/casadi_functions/'
        q_0 = [0, 0, np.pi/4, -np.pi/2, 0, np.pi/2, 0] # 7 DOF
    elif "6dof" in robot_name:
        urdf_path = os.path.join(os.path.dirname(__file__), 'fr3_6dof.urdf')
        s_functions_path = './s_functions/fr3_6dof/'
        q_0 = [0, 0, -np.pi/2, 0, np.pi/2, 0] # joint q3 fixed at pi/4
    else:
        raise Exception("Unknown robot name '{}'".format(robot_name), "Available robots: 'fr3_7dof', 'fr3_6dof'")
elif robot_name == "ur5e":
    s_functions_path = './s_functions/ur5e_6dof/casadi_functions/'
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_ur5e')
    urdf_path = os.path.join(os.path.dirname(__file__), 'ur5e.urdf')
    root_link = "base_link"
    end_link = "ur5e_tcp"
    q_0 = [0, 0, -np.pi/2, 0, -np.pi/2, 0] # standing pose
else:
    raise Exception("Unknown robot name '{}'".format(robot_name), "Available robots: 'fr3_7dof', 'fr3_6dof', 'ur5e'")

# Load robot model
robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(str(urdf_path), package_dirs=[mesh_dir])
pin_model = robot.model

# Create casadi model and data
casadi_model = cpin.Model(pin_model)
cdata = casadi_model.createData()

n = casadi_model.nq
u = cs.SX.sym('u', n, 1)
q = cs.SX.sym('q', n, 1)
q_p = cs.SX.sym('q_p', n, 1)
q_pp = cs.SX.sym('q_pp', n, 1)
x = cs.vertcat(q, q_p)

# get endeffector id
endEffector_ID = casadi_model.getFrameId(end_link)

# forward kinematics of TCP
cpin.framesForwardKinematics(casadi_model, cdata, q)

f_e = cs.Function('f_e', [q], [cdata.oMf[endEffector_ID].translation], ['q'], ['f_e'])
H_SX = cdata.oMf[endEffector_ID].homogeneous
H = cs.Function('H', [q], [H_SX], ['q'], ['H(q)'])

quat7dim_SX = cpin.SE3ToXYZQUAT(cdata.oMf[endEffector_ID])
quat_SX = cs.vertcat(quat7dim_SX[6], quat7dim_SX[3:6])
quat = cs.Function('quat', [q], [quat_SX], ['q'], ['quat(q)'])

cpin.forwardKinematics(casadi_model, cdata, q, q_p, q_pp)
# cpin.forwardDynamics(casadi_model, cdata, q, q_p, u)
cpin.updateFramePlacements(casadi_model, cdata)

C_SX = cpin.computeCoriolisMatrix(casadi_model, cdata, q, q_p) # echte 7x7 Coriolismatrix, cbra
C = cs.Function('C', [q, q_p], [C_SX], ['q', 'q_p'], ['C(q, q_p)']) # coriolis matrix

g_SX_g = cpin.computeGeneralizedGravity(casadi_model, cdata, q)
g_v2 = cs.Function('g', [q], [g_SX_g], ['q'], ['g(q)']) 
g_SX = cpin.rnea(casadi_model, cdata, q, cs.SX(n,1), cs.SX(n,1)) # tau = M*0 + C*0 + g = g
g = cs.Function('g', [q], [g_SX], ['q'], ['g(q)']) # ist beides rnea, siehe pin doku

M_SX = cpin.crba(casadi_model, cdata, q)
M = cs.Function('M', [q], [M_SX], ['q'], ['M(q)']) # inertia matrix

tau_SX = cpin.rnea(casadi_model, cdata, q, q_p, q_pp) # INV DYN: tau = M(q)q_pp + C(q, q_p)q_p + g(q) = M(q)q_pp + C_rnea(q, q_p)
inv_dyn_tau = cs.Function('inv_dyn', [q, q_p, q_pp], [tau_SX], ['q', 'q_p', 'q_pp'], ['tau(q, q_p, q_pp)'])

C_rnea_SX = cpin.rnea(casadi_model, cdata, q, q_p, cs.SX(n,1))# - g_SX # Nur so stimmt es mit Maple überein!
C_rnea = cs.Function('C_rnea', [q, q_p], [C_rnea_SX], ['q', 'q_p'], ['C_rnea(q, q_p) = C(q, q_p)q_p + g(q)']) # = n(q, q_p) = C(q, q_p)q_p + g(q)
C_rnea_v2 = cs.Function('C_rnea_v2', [q, q_p], [C(q, q_p)@q_p + g(q)], ['q', 'q_p'], ['C_rnea(q, q_p) = C(q, q_p)q_p + g(q)']) # = n(q, q_p) = C(q, q_p)q_p + g(q)

M_mat = cs.SX(n,n)
for i in range(n):
    q_pp_vec = cs.SX(n,1)
    q_pp_vec[i] = 1
    M_mat[:, i] = inv_dyn_tau(q, cs.SX(n,1), q_pp_vec) - g(q) # methode nach ott
M_v2 = cs.Function('M', [q], [cs.simplify(M_mat)], ['q'], ['M(q)']) # inertia matrix

tic = time.time()
for _ in range(10000): 
#    M_v2(q_0)
#    C_rnea(q_0, q_0) # C_rnea: 0.22s, C_rnea_v2 (cbra): 0.39s
     g_v2(q_0)
toc = time.time()
runtime = toc - tic
print(runtime)
# Wieso ist M(q) mit cbra schneller als M_v2(q) mit rnea? (M: 0.177s, M_v2: 0.288s)

M_inv_SX = cpin.computeMinverse(casadi_model, cdata, q)
M_inv = cs.Function('M_inv', [q], [M_inv_SX], ['q'], ['M_inv(q)']) # inverse inertia matrix

# Jacobian of TCP
J_SX = cpin.computeFrameJacobian(casadi_model, cdata, q, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
J = cs.Function('J', [q], [J_SX], ['q'], ['J(q)'])

# alles irgwas
#J_p_SX = cpin.computeFrameJacobianTimeVariation(casadi_model, cdata, q, q_p, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
#J_p_SX = cpin.getFrameJacobianTimeVariation(casadi_model, cdata, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
#J_p_SX = cpin.frameJacobianTimeVariation(casadi_model, cdata, q, q_p, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

J_p_SX = cs.reshape(  cs.jacobian(J(q), q) @ q_p, 6 ,n)
J_p = cs.Function('J_p', [q, q_p], [J_p_SX], ['q', 'q_p'], ['J_p(q, q_p)'])
J_p = SX00_to_SX0(J_p, q, q_p)

q_pp_aba_SX = cpin.aba(casadi_model, cdata, q, q_p, u)
q_pp_sol_SX = cs.solve( M(q), u - C_rnea(q, q_p) ) # q_pp_aba leads to error of 1e-13, sol to 0

sys_fun_qpp_aba = cs.Function('sys_fun_qpp_aba', [q, q_p, u], [q_pp_aba_SX], ['q', 'q_p', 'tau'], ['q_pp'])
sys_fun_qpp_sol = cs.Function('sys_fun_qpp_sol', [q, q_p, u], [q_pp_sol_SX], ['q', 'q_p', 'tau'], ['q_pp'])

sys_fun_x_aba = cs.Function('sys_fun_x_aba', [x, u], [cs.vertcat(x[n:2*n], sys_fun_qpp_aba(x[0:n], x[n:2*n], u))], ['x', 'u'], ['d/dt x = f(x, u) (aba)'])
sys_fun_x_sol = cs.Function('sys_fun_x_sol', [x, u], [cs.vertcat(x[n:2*n], sys_fun_qpp_sol(x[0:n], x[n:2*n], u))], ['x', 'u'], ['d/dt x = f(x, u) (sol)'])

robot_model_bus_fun = cs.Function('robot_model_bus_fun', [q, q_p], [H(q), J(q), J_p(q, q_p), M(q), C_rnea(q, q_p), g(q)], ['q', 'q_p'], ['H(q)', 'J(q)', 'J_p(q, q_p)', 'M(q)', 'n(q, q_p) = C(q, q_p)q_p + g(q)', 'g(q)'])

# get hom. transformation matrices of all joints
joint_names = casadi_model.names.tolist() # first joint is 'universal_joint' (ignore it)

for i in range(n):
    print("Joint: ", joint_names[i+1]) # ignore first joint "universal_joint"
    joint_id = casadi_model.getFrameId(joint_names[i+1])
    H_qi_SX = cdata.oMf[joint_id].homogeneous
    H_qi = cs.Function('H_q'+str(i+1), [q], [H_qi_SX], ['q'], ['H_q'+str(i+1)+'(q)'])
    H_qi.save(s_functions_path + 'hom_transform_joint_'+str(i+1)+'_py.casadi')

    J_qi_SX = cpin.computeFrameJacobian(casadi_model, cdata, q, joint_id, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_qi = cs.Function('J_q'+str(i+1), [q], [J_qi_SX], ['q'], ['J_q'+str(i+1)+'(q)'])

M.save(s_functions_path + 'inertia_matrix_py.casadi')
M_inv.save(s_functions_path + 'inverse_inertia_matrix_py.casadi')
C_rnea.save(s_functions_path + 'n_q_coriols_qp_plus_g_py.casadi')
g.save(s_functions_path + 'gravitational_forces_py.casadi')
H.save(s_functions_path + 'hom_transform_endeffector_py.casadi')
quat.save(s_functions_path + 'quat_endeffector_py.casadi')
J.save(s_functions_path + 'geo_jacobian_endeffector_py.casadi')
J_p.save(s_functions_path + 'geo_jacobian_endeffector_p_py.casadi')

sys_fun_qpp_aba.save(s_functions_path + 'sys_fun_qpp_aba_py.casadi')
sys_fun_qpp_sol.save(s_functions_path + 'sys_fun_qpp_sol_py.casadi')
sys_fun_x_aba.save(s_functions_path + 'sys_fun_x_aba_py.casadi')
sys_fun_x_sol.save(s_functions_path + 'sys_fun_x_sol_py.casadi')

inv_dyn_tau.save(s_functions_path + 'compute_tau_py.casadi')
robot_model_bus_fun.save(s_functions_path + 'robot_model_bus_fun_py.casadi')

print('Done! Run \'compile_py_cfun_to_sfun.m\' to generate S-functions.')