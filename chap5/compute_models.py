"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        2/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.rotations import Euler2Quaternion, Quaternion2Euler
import parameters.aerosonde_parameters as MAV
from parameters.simulation_parameters import ts_simulation as Ts
from message_types.msg_delta import MsgDelta


def compute_model(mav, trim_state, trim_input):
    A_lon, B_lon, A_lat, B_lat = compute_ss_model(mav, trim_state, trim_input)
    Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, \
    a_V1, a_V2, a_V3 = compute_tf_model(mav, trim_state, trim_input)

    # write transfer function gains to file
    file = open('model_coef.py', 'w')
    file.write('import numpy as np\n')
    file.write('x_trim = np.array([[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]]).T\n' %
               (trim_state.item(0), trim_state.item(1), trim_state.item(2), trim_state.item(3),
                trim_state.item(4), trim_state.item(5), trim_state.item(6), trim_state.item(7),
                trim_state.item(8), trim_state.item(9), trim_state.item(10), trim_state.item(11),
                trim_state.item(12)))
    file.write('u_trim = np.array([[%f, %f, %f, %f]]).T\n' %
               (trim_input.elevator, trim_input.aileron, trim_input.rudder, trim_input.throttle))
    file.write('Va_trim = %f\n' % Va_trim)
    file.write('alpha_trim = %f\n' % alpha_trim)
    file.write('theta_trim = %f\n' % theta_trim)
    file.write('a_phi1 = %f\n' % a_phi1)
    file.write('a_phi2 = %f\n' % a_phi2)
    file.write('a_theta1 = %f\n' % a_theta1)
    file.write('a_theta2 = %f\n' % a_theta2)
    file.write('a_theta3 = %f\n' % a_theta3)
    file.write('a_V1 = %f\n' % a_V1)
    file.write('a_V2 = %f\n' % a_V2)
    file.write('a_V3 = %f\n' % a_V3)
    file.write('A_lon = np.array([\n    [%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f]])\n' %
    (A_lon[0][0], A_lon[0][1], A_lon[0][2], A_lon[0][3], A_lon[0][4],
     A_lon[1][0], A_lon[1][1], A_lon[1][2], A_lon[1][3], A_lon[1][4],
     A_lon[2][0], A_lon[2][1], A_lon[2][2], A_lon[2][3], A_lon[2][4],
     A_lon[3][0], A_lon[3][1], A_lon[3][2], A_lon[3][3], A_lon[3][4],
     A_lon[4][0], A_lon[4][1], A_lon[4][2], A_lon[4][3], A_lon[4][4]))
    file.write('B_lon = np.array([\n    [%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f]])\n' %
    (B_lon[0][0], B_lon[0][1],
     B_lon[1][0], B_lon[1][1],
     B_lon[2][0], B_lon[2][1],
     B_lon[3][0], B_lon[3][1],
     B_lon[4][0], B_lon[4][1],))
    file.write('A_lat = np.array([\n    [%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f]])\n' %
    (A_lat[0][0], A_lat[0][1], A_lat[0][2], A_lat[0][3], A_lat[0][4],
     A_lat[1][0], A_lat[1][1], A_lat[1][2], A_lat[1][3], A_lat[1][4],
     A_lat[2][0], A_lat[2][1], A_lat[2][2], A_lat[2][3], A_lat[2][4],
     A_lat[3][0], A_lat[3][1], A_lat[3][2], A_lat[3][3], A_lat[3][4],
     A_lat[4][0], A_lat[4][1], A_lat[4][2], A_lat[4][3], A_lat[4][4]))
    file.write('B_lat = np.array([\n    [%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f]])\n' %
    (B_lat[0][0], B_lat[0][1],
     B_lat[1][0], B_lat[1][1],
     B_lat[2][0], B_lat[2][1],
     B_lat[3][0], B_lat[3][1],
     B_lat[4][0], B_lat[4][1],))
    file.write('Ts = %f\n' % Ts)
    file.close()


def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    mav._state = trim_state
    mav._update_velocity_data()
    Va_trim = mav._Va
    alpha_trim = mav._alpha
    phi, theta_trim, psi = Quaternion2Euler(trim_state[6:10])

    # define transfer function constants
    a_phi1 = -0.5*MAV.rho*(Va_trim**2)*MAV.S_wing*MAV.b*MAV.C_p_p*MAV.b/(2*Va_trim)
    a_phi2 = -0.5*MAV.rho*(Va_trim**2)*MAV.S_wing*MAV.b*MAV.C_p_delta_a
    a_theta1 = (-0.5*MAV.rho*(Va_trim**2)*MAV.c*MAV.S_wing/MAV.Jy)*MAV.C_m_q*MAV.c/(2*Va_trim)
    a_theta2 = (-0.5*MAV.rho*(Va_trim**2)*MAV.c*MAV.S_wing/MAV.Jy)*MAV.C_m_alpha
    a_theta3 = (-0.5*MAV.rho*(Va_trim**2)*MAV.c*MAV.S_wing/MAV.Jy)*MAV.C_m_delta_e

    # Compute transfer function coefficients using new propulsion model
    dTpdVa = dT_dVa(mav, Va_trim, trim_input.throttle)
    dTpdDeltaT = dT_ddelta_t(mav, Va_trim, trim_input.throttle)

    a_V1 = (MAV.rho*Va_trim*MAV.S_wing/MAV.mass)*(MAV.C_D_0 + MAV.C_D_alpha*alpha_trim + MAV.C_D_delta_e*trim_input.elevator) + dTpdVa/MAV.mass
    a_V2 = dTpdDeltaT/MAV.mass
    a_V3 = MAV.gravity*np.cos(theta_trim - alpha_trim)

    return Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_V1, a_V2, a_V3


def compute_ss_model(mav, trim_state, trim_input):
    x_euler = euler_state(trim_state)
    A = f_euler(mav, x_euler, trim_input)
    B = 0
    print("A: ", A)
    print("B: ", B)
    # extract longitudinal states (u, w, q, theta, pd) and change pd to h
    A_lon = 0
    B_lon = 0
    # extract lateral states (v, p, r, phi, psi)
    A_lat = 0
    B_lat = 0
    return A_lon, B_lon, A_lat, B_lat

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    phi, theta, psi = Quaternion2Euler(x_quat[6:10])
    x_euler = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    x_euler[0] = x_quat[0]
    x_euler[1] = x_quat[1]
    x_euler[2] = x_quat[2]
    x_euler[3] = x_quat[3]
    x_euler[4] = x_quat[4]
    x_euler[5] = x_quat[5]
    x_euler[6] = phi
    x_euler[7] = theta
    x_euler[8] = psi
    x_euler[9] = x_quat[10]
    x_euler[10] = x_quat[11]
    x_euler[11] = x_quat[12]

    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    phi = x_euler[6]
    theta = x_euler[7]
    psi = x_euler[8]
    euler = Euler2Quaternion(phi, theta, psi)

    x_quat = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    x_quat[0] = x_euler[0]
    x_quat[1] = x_euler[1]
    x_quat[2] = x_euler[2]
    x_quat[3] = x_euler[3]
    x_quat[4] = x_euler[4]
    x_quat[5] = x_euler[5]
    x_quat[6] = euler[0]
    x_quat[7] = euler[1]
    x_quat[8] = euler[2]
    x_quat[9] = euler[3]
    x_quat[10] = x_euler[9]
    x_quat[11] = x_euler[10]
    x_quat[12] = x_euler[11]

    return x_quat

def dTe_dxq(x_euler): # quat values not saved to floats.....?
    eps = 0.001
    dTeDxq = np.eye(12, 13) 
    x_quat = quaternion_state(x_euler)
    quat = x_quat[6:10]

    quat0 = np.array(quat)
    quat1 = np.array(quat)
    quat2 = np.array(quat)
    quat3 = np.array(quat)

    quat0[0] = quat[0] + eps
    quat1[1] = quat[1] + eps
    quat2[2] = quat[2] + eps
    quat3[3] = quat[3] + eps


    phi, theta, psi = Quaternion2Euler(quat)
    phi0, theta0, psi0 = Quaternion2Euler(quat0)
    phi1, theta1, psi1 = Quaternion2Euler(quat1)
    phi2, theta2, psi2 = Quaternion2Euler(quat2)
    phi3, theta3, psi3 = Quaternion2Euler(quat3)


    dQ2Ede = np.array([[(phi0 - phi)/eps,(phi1 - phi)/eps,(phi2 - phi)/eps,(phi3 - phi)/eps],[(theta0 - theta)/eps,(theta1 - theta)/eps,(theta2 - theta)/eps,(theta3 - theta)/eps],[(psi0 - psi)/eps,(psi1 - psi)/eps,(psi2 - psi)/eps,(psi3 - psi)/eps]])
    dTeDxq[6] = np.array([0,0,0,0,0,0,dQ2Ede[0][0],dQ2Ede[0][1],dQ2Ede[0][2],dQ2Ede[0][3],0,0,0])
    dTeDxq[7] = np.array([0,0,0,0,0,0,dQ2Ede[1][0],dQ2Ede[1][1],dQ2Ede[1][2],dQ2Ede[1][3],0,0,0])
    dTeDxq[8] = np.array([0,0,0,0,0,0,dQ2Ede[2][0],dQ2Ede[2][1],dQ2Ede[2][2],dQ2Ede[2][3],0,0,0])
    dTeDxq[9] = np.array([0,0,0,0,0,0,0,0,0,0,1,0,0])
    dTeDxq[10] = np.array([0,0,0,0,0,0,0,0,0,0,0,1,0])
    dTeDxq[11] = np.array([0,0,0,0,0,0,0,0,0,0,0,0,1])
    return dTeDxq

def f_euler(mav, x_euler, delta):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state
    x_quat = quaternion_state(x_euler)
    print("x_quat: ", x_quat)
    print("forces: ", mav._forces_moments(delta) )
    f_quat = mav._derivatives(x_quat, mav._forces_moments(delta))
    print("f_quat: ", f_quat)
    dTeDxq = dTe_dxq(x_euler)
    f_euler_ = dTeDxq@f_quat
    return f_euler_

def df_dx(mav, x_euler, delta):
    # take partial of f_euler with respect to x_euler
    A = 0
    return A


def df_du(mav, x_euler, delta):
    # take partial of f_euler with respect to input
    B = 0
    return B

def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    eps = 0.001
    T_eps, Q_eps = mav._motor_thrust_torque(Va + eps, delta_t)
    T, Q = mav._motor_thrust_torque(Va, delta_t)
    old_Va = Va
    return (T_eps - T) / eps

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    eps = 0.001
    T_eps, Q_eps = mav._motor_thrust_torque(Va, delta_t + eps)
    T, Q = mav._motor_thrust_torque(Va, delta_t)
    return (T_eps - T) / eps