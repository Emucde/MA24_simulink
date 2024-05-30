import time
import unittest
import os
import numpy as np
import casadi as cs
import pinocchio as pin
from pinocchio import casadi as cpin

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

cpin.forwardKinematics(casadi_model, cdata, q, q_p, q_pp)
# cpin.forwardDynamics(casadi_model, cdata, q, q_p, u)
cpin.updateFramePlacements(casadi_model, cdata)
q_pp_aba_SX = cpin.aba(casadi_model, cdata, q, q_p, u)
sys_fun_qpp = cs.Function('sys_fun_qpp', [q, q_p, u], [q_pp_aba_SX], ['q', 'q_p', 'tau'], ['q_pp'])

C_SX = cpin.computeCoriolisMatrix(casadi_model, cdata, q, q_p) # echte 7x7 Coriolismatrix
C = cs.Function('C', [q, q_p], [C_SX], ['q', 'q_p'], ['C(q, q_p)']) # coriolis matrix

g_SX = cpin.computeGeneralizedGravity(casadi_model, cdata, q)
g = cs.Function('g', [q], [g_SX], ['q'], ['g(q)'])

M_SX = cpin.crba(casadi_model, cdata, q)
M = cs.Function('M', [q], [M_SX], ['q'], ['M(q)']) # inertia matrix

joint_names = casadi_model.joint_names  # names of the joints
print(joint_names)


print('fin')
M_SX = casadi_model.inertia()
g_SX = casadi_model.gravity()
#bias_force = casadi_model.bias_force
#C_SX = casadi_model.coriolis_matrix() # not implemented!
C_rnea_SX = - casadi_model.coriolis() + g_SX # Nur so stimmt es mit Maple überein!

casadi_model.add_body([end_link])
# casadi_model.bodies # show models
J_SX = casadi_model.body(end_link).jacobian.world_aligned
J = cs.Function('J', [q], [J_SX], ['q'], ['J(q)'])

# Leider ist d/dt J völlig falsch...
#J_p_SX = casadi_model.body(end_link).jacobian_dt.world_aligned
#J_p = cs.Function('J_p', [q, q_p], [J_p_SX], ['q', 'q_p'], ['J_p(q, q_p)'])
J_p = cs.Function('J_p', [q, q_p], [cs.reshape(  cs.jacobian(J(q), q) @ q_p, 6 ,n)], ['q', 'q_p'], ['J_p(q, q_p)']) # derivative of geometric jacobian
J_p = SX00_to_SX0(J_p, q, q_p)

p_e_SX = casadi_model.body(end_link).position
R_e_SX = casadi_model.body(end_link).rotation
H = cs.Function('H', [q], [cdata.oMf[endEffector_ID].homogeneous], ['q'], ['H(q)']) # homogenious transformation matrix

quat_q4123_SX = casadi_model.body(end_link).quaternion
quat_e = cs.Function('quat_e', [q], [cs.vertcat(quat_q4123_SX[3], quat_q4123_SX[0:3])], ['q'], ['quat_e(q)']) # order is quat_e(q) = [q1, q2, q3, q4]

q_0 = [0, 0, np.pi/4, -np.pi/2, 0, np.pi/2, 0]

# Achtung der Gravitiy Vektor schaut default mäßig in [0, 0, -9.81]. Eigentlich sollte das Passen, aber scheinbar kommt es zu einen
# Vorzeichenfehler im Vergleich zu den Maple berechnungen, in denen ich den Gravity Vektor ebenfalls auf [0, 0, -9.81] gesetzt habe.
inv_dyn_tau = cs.Function('inv_dyn', [q, q_p, q_pp], [casadi_model.inverse_dynamics()], ['q', 'q_p', 'q_pp'], ['tau(q, q_p, q_pp)'])

C_rnea = cs.Function('C_rnea', [q, q_p], [C_rnea_SX], ['q', 'q_p'], ['C_rnea(q, q_p) = C(q, q_p)q_p + g(q)']) # coriolis matrix

#g = cs.Function('g', [q], [inv_dyn_tau(q, cs.SX(n,1), cs.SX(n,1))], ['q'], ['g(q)'])
g = cs.Function('g', [q], [g_SX], ['q'], ['g(q)'])

M = cs.Function('M', [q], [M_SX], ['q'], ['M(q)']) # inertia matrix

M_mat = cs.SX(n,n)

for i in range(n):
    q_pp_vec = cs.SX(n,1)
    q_pp_vec[i] = 1
    M_mat[:, i] = inv_dyn_tau(q, cs.SX(n,1), q_pp_vec) - g(q) # methode nach ott
M_v2 = cs.Function('M', [q], [M_mat], ['q'], ['M(q)']) # inertia matrix

# tic = time.time()
# for _ in range(10000):
#     M_v2(q_0)
# toc = time.time()
# runtime = toc - tic
# print(runtime)


q_pp_aba_SX = casadi_model.forward_dynamics()

print(sys_fun_qpp(q_0, q_0, q_0))
print([-21.0914, -2.73093, 18.9557, 12.8435, 9.98295, 44.4745, 9.12496]) # aus Maple

sys_fun_x = cs.Function('sys_fun_x', [x, u], [cs.vertcat(x[n:2*n], sys_fun_qpp(x[0:n], x[n:2*n], u))], ['x', 'u'], ['d/dt x = f(x, u)'])

robot_model_bus_fun = cs.Function('robot_model_bus_fun', [q, q_p], [H(q), J(q), J_p(q, q_p), M(q), C_rnea(q, q_p), g(q)], ['q', 'q_p'], ['H(q)', 'J(q)', 'J_p(q, q_p)', 'M(q)', 'n(q, q_p) = C(q, q_p)q_p + g(q)', 'g(q)'])

M.save('./s_functions/s_functions_7dof/inertia_matrix_py.casadi')
C_rnea.save('./s_functions/s_functions_7dof/n_q_coriols_qp_plus_g_py.casadi')
g.save('./s_functions/s_functions_7dof/gravitational_forces_py.casadi')
H.save('./s_functions/s_functions_7dof/hom_transform_endeffector_py.casadi')
quat_e.save('./s_functions/s_functions_7dof/quat_endeffector_py.casadi')
J.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_py.casadi')
J_p.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_p_py.casadi')

sys_fun_qpp.save('./s_functions/s_functions_7dof/sys_fun_qpp_py.casadi')
sys_fun_x.save('./s_functions/s_functions_7dof/sys_fun_x_py.casadi')
inv_dyn_tau.save('./s_functions/s_functions_7dof/compute_tau_py.casadi')
robot_model_bus_fun.save('./s_functions/s_functions_7dof/robot_model_bus_fun_py.casadi')