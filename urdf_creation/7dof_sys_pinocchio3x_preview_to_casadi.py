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

mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files')
urdf_path = os.path.join(os.path.dirname(__file__), 'fr3.urdf')

mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files')

q_0 = [0, 0, np.pi/4, -np.pi/2, 0, np.pi/2, 0]
robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(str(urdf_path))
pin_model = robot.model

casadi_model = cpin.Model(pin_model)
cdata = casadi_model.createData()

root_link = "fr3_link0"
end_link = "fr3_hand_tcp"

n = casadi_model.nq
u = cs.SX.sym('u', n, 1)
q = cs.SX.sym('q', n, 1)
q_p = cs.SX.sym('q_p', n, 1)
q_pp = cs.SX.sym('q_pp', n, 1)
x = cs.vertcat(q, q_p)

cpin.framesForwardKinematics(casadi_model, cdata, q)
endEffector_ID = casadi_model.getFrameId(end_link)

f_e = cs.Function('f_e', [q], [cdata.oMf[endEffector_ID].translation], ['q'], ['f_e'])
H_SX = cdata.oMf[endEffector_ID].homogeneous
H = cs.Function('H', [q], [H_SX], ['q'], ['H(q)'])

quat7dim_SX = cpin.SE3ToXYZQUAT(cdata.oMf[endEffector_ID])
quat_SX = cs.vertcat(quat7dim_SX[6], quat7dim_SX[3:6])
quat = cs.Function('quat', [q], [quat_SX], ['q'], ['quat(q)'])

cpin.forwardKinematics(casadi_model, cdata, q, q_p, q_pp)
# cpin.forwardDynamics(casadi_model, cdata, q, q_p, u)
cpin.updateFramePlacements(casadi_model, cdata)
q_pp_aba_SX = cpin.aba(casadi_model, cdata, q, q_p, u)
sys_fun_qpp = cs.Function('sys_fun_qpp', [q, q_p, u], [q_pp_aba_SX], ['q', 'q_p', 'tau'], ['q_pp'])

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



J_SX = cpin.computeFrameJacobian(casadi_model, cdata, q, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
J = cs.Function('J', [q], [J_SX], ['q'], ['J(q)'])

# alles irgwas
#J_p_SX = cpin.computeFrameJacobianTimeVariation(casadi_model, cdata, q, q_p, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
#J_p_SX = cpin.getFrameJacobianTimeVariation(casadi_model, cdata, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
#J_p_SX = cpin.frameJacobianTimeVariation(casadi_model, cdata, q, q_p, endEffector_ID, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

J_p_SX = cs.reshape(  cs.jacobian(J(q), q) @ q_p, 6 ,n)
J_p = cs.Function('J_p', [q, q_p], [J_p_SX], ['q', 'q_p'], ['J_p(q, q_p)'])
J_p = SX00_to_SX0(J_p, q, q_p)


sys_fun_x = cs.Function('sys_fun_x', [x, u], [cs.vertcat(x[n:2*n], sys_fun_qpp(x[0:n], x[n:2*n], u))], ['x', 'u'], ['d/dt x = f(x, u)'])

robot_model_bus_fun = cs.Function('robot_model_bus_fun', [q, q_p], [H(q), J(q), J_p(q, q_p), M(q), C_rnea(q, q_p), g(q)], ['q', 'q_p'], ['H(q)', 'J(q)', 'J_p(q, q_p)', 'M(q)', 'n(q, q_p) = C(q, q_p)q_p + g(q)', 'g(q)'])

M.save('./s_functions/s_functions_7dof/inertia_matrix_py.casadi')
C_rnea.save('./s_functions/s_functions_7dof/n_q_coriols_qp_plus_g_py.casadi')
g.save('./s_functions/s_functions_7dof/gravitational_forces_py.casadi')
H.save('./s_functions/s_functions_7dof/hom_transform_endeffector_py.casadi')
quat.save('./s_functions/s_functions_7dof/quat_endeffector_py.casadi')
J.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_py.casadi')
J_p.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_p_py.casadi')

sys_fun_qpp.save('./s_functions/s_functions_7dof/sys_fun_qpp_py.casadi')
sys_fun_x.save('./s_functions/s_functions_7dof/sys_fun_x_py.casadi')
inv_dyn_tau.save('./s_functions/s_functions_7dof/compute_tau_py.casadi')
robot_model_bus_fun.save('./s_functions/s_functions_7dof/robot_model_bus_fun_py.casadi')