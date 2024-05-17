import casadi as cs
import os
from urdf2casadi import urdfparser as u2c


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


n = 7
u = cs.SX.sym('u', n, 1)
q = cs.SX.sym('q', n, 1)
q_p = cs.SX.sym('q_p', n, 1)

gravity = [0, 0, -9.81]
M = robot_parser.get_inertia_matrix_crba(root_link, end_link) # sind casadi SX functions!
C = robot_parser.get_coriolis_rnea(root_link, end_link)
g = robot_parser.get_gravity_rnea(root_link, end_link, gravity)

H = fk_dict["T_fk"]
H_q = H(q)
f_e = cs.Function('f_e', [q], [H_q[0:3, 3]]) # endeffector position
R_e = H_q[0:3, 0:3] # endeffector rotation matrix
R_e = cs.Function('R_e', [q], [H_q[0:3, 0:3]]) # endeffector rotation matrix
R_e_p = cs.reshape(  cs.jacobian(R_e, q) @ q_p  ,3,3)  # d/dt R_e

# J_t = f_e.jacobian()
J_t = cs.Function('J_t', [q], [cs.jacobian(f_e(q), q)]) # ist equivalent

quat = fk_dict["quaternion_fk"] # dual_quaternion_fk

print(H([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5]))
print(quat([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5]))


# solve Ax = b with
# A = M(x1), b = u-C(x)*x2-g(x1)
q_pp = cs.solve( M(q), u - C(q,q_p)*q_p - g(q) ) # = d^2/dt^2 q
sys_fun = cs.Function('sys_fun', [q, q_p, u], [q_pp])

q_pp_tst = sys_fun([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

# oder
inv_dyn = robot_parser.get_inverse_dynamics_rnea(root_link, end_link) # Achtung berechnet tau!
tau = inv_dyn([0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.3, 0.3, 0.3, 0., 0.3, 0.7, 0.5], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])


sys_fun.save('./s_functions/s_functions_7dof/sys_fun_py.casadi')
M.save('./s_functions/s_functions_7dof/inertia_matrix_py.casadi')
C.save('./s_functions/s_functions_7dof/coriolis_matrix_py.casadi')
g.save('./s_functions/s_functions_7dof/gravitational_forces_py.casadi')
H.save('./s_functions/s_functions_7dof/hom_transform_endeffector.casadi')
quat.save('./s_functions/s_functions_7dof/quat_endeffector.casadi')
J.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector.casadi')
J_p.save('./s_functions/s_functions_7dof/geo_jacobian_endeffector_p.casadi')


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