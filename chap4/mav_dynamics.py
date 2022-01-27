"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
part of mavPySim 
    - Beard & McLain, PUP, 2012
    - Update history:  
        12/20/2018 - RWB
"""
from cmath import pi
from math import atan2
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import MsgState

import parameters.aerosonde_parameters as MAV
from tools.rotations import Quaternion2Rotation, Quaternion2Euler


class MavDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.north0],  # (0)
                               [MAV.east0],   # (1)
                               [MAV.down0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0]])   # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self._update_velocity_data()
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.true_state = MsgState()

    ###################################
    # public functions
    def update(self, delta, wind):
        """
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        """
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_true_state()

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pos_dot = Quaternion2Rotation(self._state[6:10]) 
        north_dot = u*(e1**2 + e0**2 - e2**2 - e3**2) + v*2*(e1*e2 - e3*e0) + w*2*(e1*e3 + e2*e0)
        east_dot = u*2*(e1*e2 + e3*e0) +  v*(e2**2 + e0**2 - e1**2 - e3**2) + w*2*(e2*e3 - e1*e0)
        down_dot = u*2*(e1*e3 - e2*e0) + v*2*(e2*e3 + e1*e0) + w*(e3**2 + e0**2 - e1**2 - e2**2)

        # position dynamics
        u_dot = r*v - q*w + fx/MAV.mass
        v_dot = q*w - r*u + fy/MAV.mass
        w_dot = q*u - p*v + fz/MAV.mass

        # rotational kinematics
        e0_dot = (-e1*p - e2*q - e3*r)/2
        e1_dot = (e0*p + e2*r - e3*q)/2
        e2_dot = (e0*q - e1*r + e3*p)/2
        e3_dot = (e0*r + e1*q - e2*p)/2

        # rotational dynamics
        p_dot = MAV.gamma1*p*q - MAV.gamma2*q*r + MAV.gamma3*l + MAV.gamma4*n
        q_dot = MAV.gamma5*p*r - MAV.gamma6*(p**2 - r**2) + m/MAV.Jy
        r_dot = MAV.gamma7*p*q - MAV.gamma1*q*r + MAV.gamma4*l + MAV.gamma8*n

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        rot = Quaternion2Rotation(self._state[6:10])
        # convert wind vector from world to body frame and add gust
        wind_body_frame = rot@steady_state + gust
        # velocity vector relative to the airmass
        v_air = [0,0,0] #??????????????????????????????????????????????????????????????????
        ur = v_air[0] - wind_body_frame[0]
        vr = v_air[0] - wind_body_frame[0]
        wr = v_air[0] - wind_body_frame[0]
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        if ur == 0:
            self._alpha = 0 #????????????????????????????????????????????????????????????
        else:
            self._alpha = atan2(wr, ur)
        # compute sideslip angle
        if self._Va == 0:
            self._beta =  0 #?????????????????????????????????????????????????????????????
        else:
            self._beta = np.asin(vr/self._Va)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        phi, theta, psi = Quaternion2Euler(self._state[6:10])

        # compute gravitaional forces
        f_g = [-MAV.mass*MAV.gravity*np.sin(theta), MAV.mass*MAV.gravity*np.cos(theta)*np.sin(phi), MAV.mass*MAV.gravity*cos(theta)*cos(phi)]

        # compute Lift and Drag coefficients
        CL = MAV.C_L_0 + MAV.C_L_alpha*self._alpha
        CD = MAV.C_D_0 + MAV.C_D_alpha*self._alpha
        # compute Lift and Drag Forces
        F_lift = 0.5*MAV.rho*self._Va**2*MAV.S_wing*(CL + MAV.C_L_q*MAV.c*q/(2*self._Va) + MAV.C_L_delta_e*delta[0])#4.6
        F_drag =  0.5*MAV.rho*self._Va**2*MAV.S_wing*(CD + MAV.C_D_q*MAV.c*q/(2*self._Va) + MAV.C_D_delta_e*delta[0]) #4.7

        #compute propeller thrust and torque
        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta[3])

        # compute longitudinal forces in body frame
        fx = f_g[0] + thrust_prop + F_drag
        fz = f_g[2] + F_lift

        # compute lateral forces in body frame
        Y_stability = 0.5*MAV.rho*self._Va**2*MAV.S_wing*(MAV.C_Y_0 + MAV.C_Y_beta*self._beta + MAV.C_Y_p*MAV.b*p/(2*self._Va) + MAV.C_Y_r*MAV.b*r/(2*self._Va) + MAV.C_Y_delta_a*delta[1] + MAV.C_Y_delta_r*delta[2])
        fy = f_g[1] + Y_stability

        # compute logitudinal torque in body frame
        My = 
        # compute lateral torques in body frame
        Mx = 
        Mz = 

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller  (See addendum by McLain) SEE PAGE 58 in ONLINE BOOK 
        # map delta_t throttle command(0 to 1) into motor input voltage
        V_in = MAV.V_max*delta_t

        # Angular speed of propeller
        a = MAV.rho*MAV.D_prop**5*MAV.C_Q0/(2*np.pi)**2
        b = MAV.rho*MAV.D_prop**4*MAV.C_Q1*Va/(2*np.pi) + MAV.KQ*MAV.KV/MAV.R_motor
        c = MAV.rho*MAV.D**3*MAV.C_Q2*Va**2 - MAV.KQ*V_in/MAV.R_motor + MAV.KQ*MAV.i0
        Omega_p = (-b + np.sqrt(b**2 - 4*a*c))/(2*a)

        # thrust and torque due to propeller
        th_a = MAV.rho*MAV.D_prop**4*MAV.C_T0/(4*np.pi**2)
        th_b = MAV.rho*MAV.D_prop**3*MAV.C_T1*Va/(2*np.pi)
        th_c = MAV.rho*MAV.D_prop**2*MAV.C_T2*Va**2

        tr_a = MAV.rho*MAV.D_prop**5*MAV.C_Q0/(4*np.pi**2)
        tr_b = MAV.rho*MAV.D_prop**4*MAV.C_Q1*Va/(2*np.pi)
        tr_c = MAV.rho*MAV.D_prop**3*MAV.C_Q2*Va**2

        thrust_prop = th_a*Omega_p**2 + th_b*Omega_p + th_c
        torque_prop =  tr_a*Omega_p**2 + tr_b*Omega_p + tr_c
        return thrust_prop, torque_prop

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
