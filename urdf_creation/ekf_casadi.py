import time
import unittest
import os
import numpy as np
import casadi as cs
import pinocchio as pin
from pinocchio import casadi as cpin

print('running ekf_casadi.py')

def SX00_to_SX0(J_fun, q, q_p=None, q_pp=None):
    if q_p is None and q_pp is None:
        J_val = J_fun(q)
    elif q_pp is None:
        J_val = J_fun(q, q_p)
    else:
        J_val = J_fun(q, q_p, q_pp)

    for row in range(J_val.shape[0]):
        for col in range(J_val.shape[1]):
            try:
                if(J_val[row, col] == 0):
                    J_val[row, col] = 0
            except:
                pass
    if q_p is None and q_pp is None:
        return cs.Function(J_fun.name(), [q], [J_val], J_fun.name_in(), J_fun.name_out())
    elif q_pp is None:
        return cs.Function(J_fun.name(), [q, q_p], [J_val], J_fun.name_in(), J_fun.name_out())
    else:
        return cs.Function(J_fun.name(), [q, q_p, q_pp], [J_val], J_fun.name_in(), J_fun.name_out())

# robot_name = "fr3_7dof"
# robot_name = "fr3_6dof"
robot_name = "fr3_no_hand_6dof"
# robot_name = "ur5e"

if "fr3" in robot_name:
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_FR3')
    root_link = "fr3_link0"
    if robot_name == "fr3_7dof":
        urdf_path = os.path.join(os.path.dirname(__file__), 'fr3_7dof.urdf')
        s_functions_path = './s_functions/fr3_7dof/casadi_functions/'
        q_0 = [0, 0, np.pi/4, -np.pi/2, 0, np.pi/2, 0] # 7 DOF
        end_link = "fr3_hand_tcp"
    elif robot_name == "fr3_6dof":
        urdf_path = os.path.join(os.path.dirname(__file__), 'fr3_6dof.urdf')
        s_functions_path = './s_functions/fr3_6dof/casadi_functions/'
        q_0 = [0, 0, -np.pi/2, 0, np.pi/2, 0] # joint q3 fixed at pi/4
        end_link = "fr3_hand_tcp"
    elif robot_name == "fr3_no_hand_6dof":
        urdf_path = os.path.join(os.path.dirname(__file__), 'fr3_no_hand_7dof.urdf')
        s_functions_path = './s_functions/fr3_no_hand_6dof/casadi_functions/'
        q_0 = [0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4] # fixed joint later due to joint space control
        end_link = "fr3_link8_tcp"
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

gravity_configs = [True, False]
for gravity_used in gravity_configs:
    if not gravity_used:
        pin_model.gravity.linear[:] = [0, 0, 0]
        append_gravity_str = "_nogravity"
        fun_name='ekf_fun_no_gravity_py.casadi'
    else:
        append_gravity_str = ""
        fun_name='ekf_fun_py.casadi'

    # Create casadi model and data
    casadi_model = cpin.Model(pin_model)
    cdata = casadi_model.createData()

    n = casadi_model.nq
    u = cs.SX.sym('u', n, 1)
    q = cs.SX.sym('q', n, 1)
    q_p = cs.SX.sym('q_p', n, 1)
    q_pp = cs.SX.sym('q_pp', n, 1)
    x = cs.vertcat(q, q_p)
    w = cs.SX.sym('w', 2*n, 1) # process noise
    n_w = w.shape[0]

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

    g_v2_SX = cpin.computeGeneralizedGravity(casadi_model, cdata, q)
    g_v2 = cs.Function('g', [q], [g_v2_SX], ['q'], ['g(q)']) 
    g_SX = cpin.rnea(casadi_model, cdata, q, cs.SX(n,1), cs.SX(n,1)) # tau = M*0 + C*0 + g = g
    g = cs.Function('g', [q], [g_SX], ['q'], ['g(q)']) # ist beides rnea, siehe pin doku

    M_SX = cpin.crba(casadi_model, cdata, q)
    M = cs.Function('M', [q], [M_SX], ['q'], ['M(q)']) # inertia matrix

    tau_SX = cpin.rnea(casadi_model, cdata, q, q_p, q_pp) # INV DYN: tau = M(q)q_pp + C(q, q_p)q_p + g(q) = M(q)q_pp + C_rnea(q, q_p)
    tau_v2_SX = M(q)@q_pp + C(q, q_p)@q_p + g(q)
    inv_dyn_tau = cs.Function('inv_dyn', [q, q_p, q_pp], [tau_SX], ['q', 'q_p', 'q_pp'], ['tau(q, q_p, q_pp)'])
    inv_dyn_tau_v2 = cs.Function('inv_dyn', [q, q_p, q_pp], [tau_v2_SX], ['q', 'q_p', 'q_pp'], ['tau(q, q_p, q_pp)'])

    C_rnea_SX = cpin.rnea(casadi_model, cdata, q, q_p, cs.SX(n,1))# - g_SX # Nur so stimmt es mit Maple überein!
    C_rnea = cs.Function('C_rnea', [q, q_p], [C_rnea_SX], ['q', 'q_p'], ['C_rnea(q, q_p) = C(q, q_p)q_p + g(q)']) # = n(q, q_p) = C(q, q_p)q_p + g(q)
    C_rnea_v2 = cs.Function('C_rnea_v2', [q, q_p], [C(q, q_p)@q_p + g(q)], ['q', 'q_p'], ['C_rnea(q, q_p) = C(q, q_p)q_p + g(q)']) # = n(q, q_p) = C(q, q_p)q_p + g(q)

    q_pp_aba_SX = cpin.aba(casadi_model, cdata, q, q_p, u)
    q_pp_sol_SX = cs.solve( M(q), u - C_rnea(q, q_p) ) # q_pp_aba leads to error of 1e-13, sol to 0

    sys_fun_qpp_aba = cs.Function('sys_fun_qpp_aba', [q, q_p, u], [q_pp_aba_SX], ['q', 'q_p', 'tau'], ['q_pp'])
    sys_fun_qpp_sol = cs.Function('sys_fun_qpp_sol', [q, q_p, u], [q_pp_sol_SX], ['q', 'q_p', 'tau'], ['q_pp'])

    sys_fun_x_aba = cs.Function('sys_fun_x_aba', [x, u, w], [cs.vertcat(x[n:2*n] + w[:n], sys_fun_qpp_aba(x[0:n], x[n:2*n], u) + w[n:2*n])], ['x', 'u', 'w'], ['d/dt x = f(x, u) (aba)'])
    sys_fun_x_sol = cs.Function('sys_fun_x_sol', [x, u, w], [cs.vertcat(x[n:2*n] + w[:n], sys_fun_qpp_sol(x[0:n], x[n:2*n], u) + w[n:2*n])], ['x', 'u', 'w'], ['d/dt x = f(x, u) (sol)'])

    sys_fun_x = sys_fun_x_aba

    def RK4(f, x, u, w, h):
        k1 = f(x, u, w)
        k2 = f(x + h/2*k1, u, w)
        k3 = f(x + h/2*k2, u, w)
        k4 = f(x + h*k3, u, w)
        return cs.Function('F', [x, u, w], [x + h/6*(k1 + 2*k2 + 2*k3 + k4)], ['x', 'u', 'w'], ['x_next'])

    def Euler(f, x, u, w, h):
        return cs.Function('F', [x, u, w], [x + h*f(x, u, w)], ['x', 'u', 'w'], ['x_next'])

    # 1. Berechnen des Abastsystems:
    # Annahme: Die Prozessstörung tritt additiv auf, d. h.
    # tilde f(x, u, w) = f(x, u) + [0, w]
    # wk is interpreted as an unkown input to the system
    # wk wirkt auf qpp aber nicht auf qp
    # Ausgang:
    # y = x + v
    # yk = xk + vk = xk_measured
    # v ... Messrauschen

    Ta = 1e-3
    F = Euler(sys_fun_x, x, u, w, Ta)

    xk_measured = x
    # xk_measured = q
    n_m = xk_measured.shape[0]

    v = cs.SX.sym('v', n_m, 1) # measurement noise
    hk = cs.Function('hk', [x, v], [xk_measured + v], ['x', 'v'], ['hk']) # y = x + v = h(x, v) = x_measured
    dFdx = cs.Function('dFdx', [x, u, w], [cs.jacobian(F(x, u, w), x)], ['x', 'u', 'w'], ['dFdx']) # at w = 0
    dFdw = cs.Function('dFdw', [x, u, w], [cs.jacobian(F(x, u, w), w)], ['x', 'u', 'w'], ['dFdw']) # because f(x, u, w) = f(x, u) + w
    dHdx = cs.Function('dHdx', [x, v], [cs.jacobian(hk(x, v), x)], ['x', 'v'], ['dHdx']) # y = x + v = h(x, v)

    dFdx = SX00_to_SX0(dFdx, x, u, w)
    dFdw = SX00_to_SX0(dFdw, x, u, w)
    dHdx = SX00_to_SX0(dHdx, x, v)

    xk_minus = cs.SX.sym('xk_minus', 2*n, 1)
    Pk_minus = cs.SX.sym('Pk_minus', 2*n, 2*n)
    uk = cs.SX.sym('uk', n, 1)
    wk = cs.SX.sym('wk', n, 1)

    xk_measured = cs.SX.sym('xk_measured', n_m, 1)

    Rk = cs.SX.sym('Rk', n_m, n_m) # measurement noise covariance
    Qk = cs.SX.sym('Qk', n_w, n_w) # process noise covariance

    # maybe wäre es besser für yk nur die joint winkel zu verwenden, da die joint geschwindigkeiten und beschleunigungen nicht gemessen werden können
    # und sowieso nur numerisch berechnet wurden...
    yk = xk_measured # contains v, but v i s unkown

    Ck = dHdx(xk_minus, cs.SX.zeros(n_m, 1))
    uk_breve = hk(xk_minus, cs.SX.zeros(n_m, 1)) - Ck @ xk_minus # = 0, because hk(xk_minus, 0) = xk_minus

    # Lk_hat = Pk_minus @ Ck.T @ ( cs.inv(Ck @ Pk_minus @ Ck.T + Rk) )

    # L = A * B^(-1) => L * B = A
    # X A = B
    # A.T X.T = B.T => As Xs = Bs => Xs = solve(As, Bs) => X = Xs.T
    As = (Ck @ Pk_minus @ Ck.T + Rk).T # glaub cs.inv is eh äquvialent
    Bs = (Pk_minus @ Ck.T).T
    Lk_hat = (cs.solve(As, Bs)).T

    xk_plus = xk_minus + Lk_hat @ (yk - Ck @ xk_minus - uk_breve)
    Pk_plus = (cs.SX.eye(2*n) - Lk_hat @ Ck) @ Pk_minus
    Phi_k = dFdx(xk_plus, uk, cs.SX.zeros(n_w,1))
    Gk = dFdw(xk_plus, uk, cs.SX.zeros(n_w,1))
    xkp1_minus = F(xk_plus, uk, cs.SX.zeros(n_w,1))
    Pkp1_minus = Phi_k @ Pk_plus @ Phi_k.T + Gk @ Qk @ Gk.T

    ekf_fun = cs.Function('ekf_fun', [uk, yk, Rk, Qk, xk_minus, Pk_minus], \
                        [xk_plus, xkp1_minus, Pkp1_minus], \
                        ['uk ({}x1)'.format(n), 'yk ({}x1)'.format(n_m), 'Rk ({}x{})'.format(n_m, n_m), 'Qk ({}x{})'.format(2*n, 2*n), 'xk_minus ({}x1)'.format(2*n), 'Pk_minus ({}x{})'.format(2*n, 2*n)], \
                        ['xk_plus ({}x1)'.format(2*n), 'xkp1_minus ({}x1)'.format(2*n), 'Pkp1_minus ({}x{})'.format(2*n, 2*n)])

    ekf_fun.save(s_functions_path + fun_name)

    yk_test = np.zeros(n_m)
    yk_test[0:n] = q_0
    tic = time.time()
    for _ in range(100):
        ekf_fun(np.zeros(n), yk_test, cs.SX.eye(n_m), cs.SX.eye(n_w), cs.SX.zeros(2*n, 1), cs.SX.eye(2*n))
    toc = time.time()
    runtime = toc - tic
    print(runtime)

print('Done! Run \'compile_py_cfun_to_sfun.m\' to generate S-functions.')