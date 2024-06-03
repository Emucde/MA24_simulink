# Bitte conda env 'casadi_kin_dyn' verwenden, denn nur dort ist casadi_kin_dyn und darli installiert.
# https://github.com/lvjonok/casadi_kin_dyn
# https://github.com/simeon-ned/darli

# based on https://github.com/simeon-ned/darli/blob/master/tutorial/00_intro.ipynb

import time
import os
import numpy as np
import casadi as cs
from darli.backend import CasadiBackend, PinocchioBackend
from darli.model import Model

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


urdf_path = os.path.join(os.path.dirname(__file__), 'fr3.urdf')
root_link = "fr3_link0"
end_link = "fr3_hand_tcp"

# Building the Pinnochio backend
pin_backend = PinocchioBackend(urdf_path)
# pin_backend._PinocchioBackend__model.gravity.linear
# Building the CasADi backend
cas_backend = CasadiBackend(urdf_path)
#cas_backend._pinmodel.gravity.linear = np.array([0, 0, -9.81])

#pin_backend.nq
#pin_backend.nv
#pin_backend.aba
#pin_backend.rnea

# casadi_model = Model(CasadiBackend(urdf_path))
casadi_model = Model(cas_backend)

nq = casadi_model.nq  # dimensionality of configuration
nv = casadi_model.nv  # dimensionality of generilized velocities
nu = casadi_model.nu  # dimensionality  of control inputs
q_min, q_max = (
    casadi_model.q_min,
    casadi_model.q_max,
)  # minimal and maximal limits on joint pos
nq, nv, nu

n = nq
u = casadi_model._u
q = casadi_model.q
q_p = casadi_model.v
q_pp = casadi_model.dv

joint_names = casadi_model.joint_names  # names of the joints
print(joint_names)

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
H = cs.Function('H', [q], [cs.vertcat(cs.horzcat(R_e_SX, p_e_SX), cs.horzcat(0, 0, 0, 1))], ['q'], ['H(q)']) # homogenious transformation matrix

quat_q4123_SX = casadi_model.body(end_link).quaternion
quat_e = cs.Function('quat_e', [q], [cs.vertcat(quat_q4123_SX[3], quat_q4123_SX[0:3])], ['q'], ['quat_e(q)']) # order is quat_e(q) = [q1, q2, q3, q4]

# q_0 = [0, 0, np.pi/4, -np.pi/2, 0, np.pi/2, 0] # 7 DOF
q_0 = [0, 0, -np.pi/2, 0, np.pi/2, 0] # joint q3 fixed at pi/4

com_position = casadi_model.com().position
com_velocity = casadi_model.com().velocity
com_jacobian = casadi_model.com().jacobian
com_jacobian_dt = casadi_model.com().jacobian_dt
potential_energy = casadi_model.energy().potential
kinetic_energy = casadi_model.energy().kinetic
lagrangian = casadi_model.lagrangian

# nur welcher com ist das??
com_pos_fun = cs.Function('com_position', [q], [com_position], ['q'], ['com_position(q)'])

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

# M_v2 dauert ca. doppelt so lang (0.297 s vs 0.160 s) bzw. (0.997, 0.177)
tic = time.time()
for _ in range(10000):
    M_v2(q_0)
toc = time.time()
runtime = toc - tic
print(runtime)


q_pp_aba_SX = casadi_model.forward_dynamics()
# scheinbar ist die Funktion verbuggt und es wurden andere u Variablen hineingeschrieben, auch wenn
# sie gleich lauten. Casadi unterscheidet hier. Daher definiere ich die Funktion zuerst so, dass sie
# freie Variablen haben darf. Erst dann gibt es eine Funktion mit der man die freien Variablen
# auslesen kann und mit diesen wird u neu gesetzt, sodass die Funktion schließlich erstellt werden kann.

opt = dict({'allow_free': True})
sys_fun_qpp = cs.Function('sys_fun_qpp', [q, q_p, u], [q_pp_aba_SX], ['q', 'q_p', 'tau'], ['q_pp'], opt)
u_new = sys_fun_qpp.free_sx() # achtung tau0, tau6, tau5, tau4, tau3, tau2, tau1
u_new = cs.vertcat(u_new[0], u_new[6], u_new[5], u_new[4], u_new[3], u_new[2], u_new[1])
sys_fun_qpp = cs.Function('sys_fun_qpp', [q, q_p, u_new], [q_pp_aba_SX], ['q', 'q_p', 'tau'], ['q_pp'])

x = cs.vertcat(q, q_p)

#sys_fun_x = cs.Function('sys_fun_x', [x, u], [cs.vertcat(q_p, sys_fun_qpp(q, q_p, u))], ['x', 'u'], ['d/dt x = f(x, u)'])
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


print(inv_dyn_tau(q_0, 0*q_0, 0*q_0))
print('ende')