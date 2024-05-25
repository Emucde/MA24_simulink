import casadi as cs
import os
from urdf2casadi import urdfparser as u2c
import numpy as np

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

urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'fr3.urdf')
root_link = "fr3_link0"
end_link = "fr3_hand_tcp"
robot_parser = u2c.URDFparser()
robot_parser.from_file(urdf_path)
# Also supports .from_server for ros parameter server, or .from_string if you have the URDF as a string.
fk_dict = robot_parser.get_forward_kinematics(root_link, end_link)
print(fk_dict.keys())
# should give ['q', 'upper', 'lower', 'dual_quaternion_fk', 'joint_names', 'T_fk', 'joint_list', 'quaternion_fk']
#forward_kinematics = fk_dict["T_fk"]

m=6
n = 7
u = cs.SX.sym('u', n, 1)
q = cs.SX.sym('q', n, 1)
q_p = cs.SX.sym('q_p', n, 1)
q_pp = cs.SX.sym('q_pp', n, 1)
x = cs.vertcat(q, q_p)
tau = cs.SX.sym('tau', n, 1)
S = cs.SX.sym('S', 3, 3)

gravity = [0, 0, -9.81]
gravity = [-1*el for el in gravity] # *(-1): scheinbar haben sie die Z-achse nach unten zeigend angenommen!!!

inv_dyn = robot_parser.get_inverse_dynamics_rnea(root_link, end_link, gravity) # Achtung berechnet tau!
inv_dyn_tau = cs.Function('inv_dyn', [q, q_p, q_pp], [inv_dyn(q, q_p, q_pp)], ['q', 'q_p', 'q_pp'], ['tau(q, q_p, q_pp)'])

q_0 = [0, 0, np.pi/4, -np.pi/2, 0, np.pi/2, 0]
qa = [1,2,3,4,5,6,7]
qpa = [1,2,3,4,5,6,7]
print('qa = [1,2,3,4,5,6,7], qpa = [1,2,3,4,5,6,7]', '\n')

g1 = robot_parser.get_gravity_rnea(root_link, end_link, gravity)
g = cs.Function('g', [q], [inv_dyn_tau(q, cs.SX(n,1), cs.SX(n,1))], ['q'], ['g(q)']) # gravitational forces
print('g1(qa) - g(qa):', g1(qa) - g(qa), '\n') # sollte 0 sein

M_mat = cs.SX(n,n)

for i in range(n):
    q_pp_vec = cs.SX(n,1)
    q_pp_vec[i] = 1
    M_mat[:, i] = inv_dyn_tau(q, cs.SX(n,1), q_pp_vec) - g(q) # methode nach ott

M1 = robot_parser.get_inertia_matrix_crba(root_link, end_link) # sind casadi SX functions! Ineffizient, siehe paper
M = cs.Function('M', [q], [M_mat], ['q'], ['M(q)']) # inertia matrix

print('M1(qa) - M(qa):', M1(qa) - M(qa), '\n') # sollte 0 sein
print('M(qa) - M(qa).T:', M(qa) - M(qa).T, '\n') # sollte 0 sein

C_rnea_wog = robot_parser.get_coriolis_rnea(root_link, end_link) # rnea algorithm!! (man erhält nur einen nx1 vektor d. h. C(q, q_p) * q_p)
C_rnea1 = cs.Function('C_rnea1', [q, q_p], [C_rnea_wog(q, q_p) + g(q)], ['q', 'q_p'], ['C_rnea1(q, q_p)']) # = C(q, q_p)q_p + g(q)
C_rnea = cs.Function('C_rnea', [q, q_p], [inv_dyn_tau(q, q_p, cs.SX(n,1))], ['q', 'q_p'], ['C(q, q_p) = tau(q, q_p, 0)']) # = n(q, q_p) = C(q, q_p)q_p + g(q)
print('C_rnea1(qa, qpa) - C_rnea(qa, qpa):', C_rnea1(qa, qpa) - C_rnea(qa, qpa)) # sollte 0 sein

C_nq = cs.Function('C_nq', [q, q_p], [C_rnea(q, q_p)], ['q', 'q_p'], ['n(q, q_p) = C(q, q_p)q_p+g(q)']) # coriolis matrix

# Die Eigenschaft C(q, q_p) + C(q, q_p).T = M kann nicht geprüft werden, da die Funktion C_rnea(q, q_p) = C(q, q_p)q_p + g(q) = n(q, q_p)
dM = cs.Function('dM', [q, q_p], [cs.reshape( cs.jacobian(M(q), q) @ q_p, n, n)], ['q', 'q_p'], ['dM(q, q_p)']) # derivative of inertia matrix
test_fun = cs.Function('test_fun', [q, q_p], [(dM(q, q_p) @ q_p - 2*(C_rnea(q, q_p) - g(q)))], ['q', 'q_p'], ['zero'])
# test_fun(qa, qpa) # sollte 0 sein (funktioniert nicht)

# Hinweis: es gibt cs.simplify()

T_fk = fk_dict["T_fk"]
H = cs.Function('H', [q], [T_fk(q)], ['q'], ['H(q)']) # homogenious transformation matrix
H_q = H(q)
f_e = cs.Function('f_e', [q], [H_q[0:3, 3]], ['q'], ['f_e(q)']) # endeffector position
R_e = cs.Function('R_e', [q], [H_q[0:3, 0:3]], ['q'], ['R_e(q)']) # endeffector rotation matrix
R_e_p = cs.Function('R_e_p', [q, q_p], [cs.reshape(  cs.jacobian(R_e(q), q) @ q_p  ,3,3)], ['q', 'q_p'], ['R_e_p(q, q_p)']) # derivative of rotation matrix

# J_t = f_e.jacobian()
J_t = cs.Function('J_t', [q], [cs.jacobian(f_e(q), q)], ['q'], ['J_t(q)']) # translational jacobian
S_omega = cs.Function('S_omega', [q, q_p], [R_e_p(q, q_p) @ R_e(q).T], ['q', 'q_p'], ['S_omega(q, q_p)'])
skew_to_vector = cs.Function('skew_to_vector', [S], [cs.vertcat(S[2,1], S[0,2], S[1,0])], ['S'], ['skew_to_vector(S)'])
omega = cs.Function('omega', [q, q_p], [skew_to_vector(S_omega(q, q_p))], ['q', 'q_p'], ['omega(q, q_p)'])
domega = cs.jacobian(omega(q, q_p), q_p) # das q_p verschwindet dabei
J_r = cs.Function('J_r', [q], [domega], ['q'], ['J_r(q)']) # rotational jacobian

J = cs.Function('J', [q], [cs.vertcat(J_t(q), J_r(q))], ['q'], ['J(q)']) # geometric jacobian
J_p = cs.Function('J_p', [q, q_p], [cs.reshape(  cs.jacobian(J(q), q) @ q_p  ,m ,n)], ['q', 'q_p'], ['J_p(q, q_p)']) # derivative of geometric jacobian

quat = fk_dict["quaternion_fk"] # dual_quaternion_fk, order is quat = [q2,q3,q4,q1]
quat_e = cs.Function('quat_e', [q], [cs.vertcat(quat(q)[3], quat(q)[0:3])], ['q'], ['quat_e(q)']) # order is quat_e(q) = [q1, q2, q3, q4]

# print(H([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5]))
# print(quat([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5]))

# solve Ax = b with
# A = M(x1), b = u-C(x)*x2-g(x1)
# Achtung: C(q,q_p)*q_p + g(q) = C_rnea(q, q_p) = C_nq(q, q_p)
q_pp_crba = cs.solve( M(q), tau - C_nq(q,q_p)) # = d^2/dt^2 q, ineffizient, vgl. robot_parser.get_forward_dynamics_crba
q_pp_aba_fun = robot_parser.get_forward_dynamics_aba(root_link, end_link, gravity) # = d^2/dt^2 q über aba berechnet

# q_pp_alg = q_pp_crba
q_pp_alg = q_pp_aba_fun(q, q_p, tau)
sys_fun_qpp = cs.Function('sys_fun_qpp', [q, q_p, tau], [q_pp_alg], ['q', 'q_p', 'tau'], ['q_pp'])

#sys_fun_x = cs.Function('sys_fun_x', [x, u], [cs.vertcat(q_p, sys_fun_qpp(q, q_p, u))], ['x', 'u'], ['d/dt x = f(x, u)'])
sys_fun_x = cs.Function('sys_fun_x', [x, u], [cs.vertcat(x[n:2*n], sys_fun_qpp(x[0:n], x[n:2*n], u))], ['x', 'u'], ['d/dt x = f(x, u)'])


q_pp_tst = sys_fun_qpp([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

tau = inv_dyn([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

## Update: Es kam zu großen Problemen, wenn SX(00) in Casadi Funktionen vorkamen. Dann waren die Reihenfolge der
## Spalten und Zeilen völlig durcheinander. Deshalb habe ich die Funktion SX00_to_SX0(J_fun, q, q_p=None), die alle
## SX(00) mit SX(0) ersetzt, wohl eine Daterntypkonvertierung. Vermutlich könnte es daran liegen, die Zeros erst am Schluss
## ausgegeben werden, aber das ist nicht klar. Bei SX(0) wird der Eintrag nicht mehr als nonzero (nz) erkannt.
## Dadurch wird z.b. mit J = SX00_to_SX0(J, q) die ursprüngliche Funktion
## Function(J:(q[7])->(J(q)[6x7,37nz]) SXFunction)
## zu
## Function(J:(q[7])->(J(q)[6x7]) SXFunction)
## konvertiert.

# H, M, C_rnea, g = SX00_to_SX0(H, q) nicht notwendig, da in H keine nz vorkommen. Das entsteht erst durch jacobimatrizen
J = SX00_to_SX0(J, q)
J_p = SX00_to_SX0(J_p, q, q_p)

robot_model_bus_fun = cs.Function('robot_model_bus_fun', [q, q_p], [H(q), J(q), J_p(q, q_p), M(q), C_nq(q, q_p), g(q)], ['q', 'q_p'], ['H(q)', 'J(q)', 'J_p(q, q_p)', 'M(q)', 'n(q, q_p) = C(q, q_p)q_p + g(q)', 'g(q)'])

M.save('./s_functions/s_functions_7dof/inertia_matrix_py.casadi')
C_nq.save('./s_functions/s_functions_7dof/n_q_coriols_qp_plus_g_py.casadi')
g.save('./s_functions/s_functions_7dof/gravitational_forces_py.casadi')
H.save('./s_functions/s_functions_7dof/hom_transform_endeffector_py.casadi')
quat.save('./s_functions/s_functions_7dof/quat_endeffector_py.casadi')
J.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_py.casadi')
J_p.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_p_py.casadi')

sys_fun_qpp.save('./s_functions/s_functions_7dof/sys_fun_qpp_py.casadi')
sys_fun_x.save('./s_functions/s_functions_7dof/sys_fun_x_py.casadi')
inv_dyn_tau.save('./s_functions/s_functions_7dof/compute_tau_py.casadi')
robot_model_bus_fun.save('./s_functions/s_functions_7dof/robot_model_bus_fun_py.casadi')


# fun_arr = { ...
#     'sys_fun_py', ...
#     'inertia_matrix_py', ...
#     'coriolis_matrix_py', ...
#     'gravitational_forces_py', ...
#     'hom_transform_endeffector', ...
#     'quat_endeffector', ...
#     'geo_jacobian_endeffector', ...
#     'geo_jacobian_endeffector_p' ...
# };