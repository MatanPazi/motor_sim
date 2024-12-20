"""
Script Name: Motor_Simulator.py
Author: Matan Pazi
Date: October 13th, 2024
Version: 1.0
Description: 
    This script simulates the behavior of a three-phase motor, including phase 
    current dynamics, PWM switching behavior, and motor torque generation.
    The simulation includes:
        - Per-phase inductance calculations
        - Dead-time compensation for switching behavior
        - DQZ transformations for current control
        - Torque ripple analysis based on harmonics in the back-EMF
        - Option to short circuit the phases

Usage:
    Set the Motor, Simulation, Application and Control classes initializations correctly.
    Choose which variables to plot.
    And run.

License:
    MIT License. See LICENSE file in the project root for details.

"""

from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import control as ctrl

class Motor:
    def __init__(self, motor_type="SYNC", pole_pairs=4, Rs=0.0028, Lq_base=0.000077, Ld_base=0.0000458,
                 bemf_const=0.11459, inertia=0.01, visc_fric_coeff=0.005, i_max = 600):
        '''
        Specifies motor-related parameters.

        Args:
            motor_type (str): Motor type for torque calculation. 
                - 'ASYNC': Induction motor.
                - 'SYNC': Synchronous motor types.
            Rs (float): Phase resistance [Ohm].
            Lq_base (float): Q-axis inductance measured at low current [H].
            Ld_base (float): D-axis inductance measured at low current [H].
            bemf_const (float): Peak line-to-line voltage measured between two phases [V/(rad/sec)].
            flux_linkage (float): Permanent magnet flux linkage [Wb].
            inertia (float): Motor inertia [kg*m^2].
            visc_fric_coeff (float): Viscous friction coefficient [Nm*s/rad].
            i_max (float): Maximum motor current [A].
            harmonics (dict or None): Harmonic selection, either `None` or a dictionary specifying relevant harmonics.
        '''
        self.motor_type = motor_type
        self.pole_pairs = pole_pairs
        self.Rs = Rs
        self.Lq_base = Lq_base
        self.Ld_base = Ld_base
        self.Lq = Lq_base
        self.Ld = Ld_base        
        self.Laa = 0
        self.Lbb = 0
        self.Lcc = 0
        self.Lab = 0
        self.Lac = 0
        self.Lbc = 0
        self.Laa_dot = 0
        self.Lbb_dot = 0
        self.Lcc_dot = 0
        self.Lab_dot = 0
        self.Lac_dot = 0
        self.Lbc_dot = 0
        self.bemf_const = bemf_const / np.sqrt(3)       # Convert from line-to-line to phase (Wye topology)
        self.bemf_a = 0
        self.bemf_b = 0
        self.bemf_c = 0        
        self.flux_linkage = bemf_const / pole_pairs / 1.5
        # Harmonics, choose preferred option (Comment out the other):
        #   None for no bemf harmonics
        #   dictionary for desired harmonics
        self.harmonics = None
        # self.harmonics = {1: {'harmonic': 5, 'mag': bemf_const / 20},
        #                   2: {'harmonic': 7, 'mag': bemf_const / 20},
        #                   3: {'harmonic': 9, 'mag': bemf_const / 40},
        #                   4: {'harmonic': 11, 'mag': bemf_const / 40}}
        self.inertia = inertia
        self.visc_fric_coeff = visc_fric_coeff
        self.i_max = i_max

        

    def inductance_dq(self, iq, id):
        """
        Update the Lq, Ld inductances based on current amplitude
        """
        Is = np.sqrt(iq**2 + id**2)  # Total current magnitude
        # Assuming inductance reduces by half at peak current.
        if (Is < self.i_max):
            self.Lq = self.Lq_base * (1 - 0.5 * Is/self.i_max)
            self.Ld = self.Ld_base * (1 - 0.5 * Is/self.i_max)
        else:
            self.Lq = self.Lq_base * 0.5
            self.Ld = self.Ld_base * 0.5
    
    def inductance_abc(self, theta):
        """
        Update the inductances in the abc frame (Based on the dq transform)
        """        
        self.Laa = self.Lq * (np.cos(theta))**2 + self.Ld * (np.sin(theta))**2
        self.Lbb = self.Lq * (np.cos(theta - 2*np.pi/3))**2 + self.Ld * (np.sin(theta - 2*np.pi/3))**2
        self.Lcc = self.Lq * (np.cos(theta + 2*np.pi/3))**2 + self.Ld * (np.sin(theta + 2*np.pi/3))**2   

        # Mutual inductance - Assuming position dependency only.
        self.Lab = self.Lq * np.cos(theta) * np.cos(theta - 2*np.pi/3) + self.Ld * np.sin(theta) * np.sin(theta - 2*np.pi/3)
        self.Lac = self.Lq * np.cos(theta) * np.cos(theta + 2*np.pi/3) + self.Ld * np.sin(theta) * np.sin(theta + 2*np.pi/3)
        self.Lbc = self.Lq * np.cos(theta - 2*np.pi/3) * np.cos(theta + 2*np.pi/3) + self.Ld * np.sin(theta - 2*np.pi/3) * np.sin(theta + 2*np.pi/3)

    def inductance_abc_dot(self, theta, speed):        
        """
        Update the inductances time derivatives
        """  
        # Derivatives for self inductances
        self.Laa_dot = -(self.Lq - self.Ld) * np.sin(2 * theta) * speed
        self.Lbb_dot = -(self.Lq - self.Ld) * np.cos(2 * theta + np.pi/6) * speed
        self.Lcc_dot = (self.Lq - self.Ld) * np.sin(2 * (theta + np.pi/6)) * speed
        
        # Derivatives for mutual inductances
        self.Lab_dot = (self.Lq - self.Ld) * np.cos(np.pi/6 - 2 * theta) * speed
        self.Lac_dot = -(self.Lq - self.Ld) * np.cos(2 * theta + np.pi/6) * speed
        self.Lbc_dot = -(self.Lq - self.Ld) * np.sin(2 * theta) * speed

    def phase_bemf(self, angle, phase_shift):
        """
        Calculate bemf, allow for harmonics.
        """          
        bemf = self.bemf_const * np.cos(angle + phase_shift)
        if self.harmonics:
            for _, data in self.harmonics.items():
                bemf += data['mag'] * np.cos(data['harmonic'] * angle + phase_shift)        
        return bemf

    def torque(self, iq, id):
        """
        Calculate Torque based on motor type: Synchronous or asynchronous
        """           
        if self.motor_type == "SYNC":
            torque = 1.5 * self.pole_pairs * (self.flux_linkage * iq + (self.Ld - self.Lq) * iq * id)
        else: # ASYNC motor
            torque = 1.5 * self.pole_pairs * (self.flux_linkage * iq + (self.Ld - self.Lq) * iq * id) # TODO: Update equation for async motors
        return torque

class Simulation:
    def __init__(self, time_step=100e-9, total_time=0.05):
        '''
        Initializes simulation related parameters:

        Args:
            time_step (float): Simualtion time step [sec]
            total_time (float): Total simulation time [sec]
        '''
        self.time_step = time_step
        self.total_time = total_time
        self.time_points = np.arange(0, total_time, time_step)

class Application:
    def __init__(self, speed_control=True, commanded_speed=100.0, commanded_iq=200.0, commanded_id=-50.0,
                 acceleration=10000.0, current_ramp=10000.0, vbus = 48, init_speed = 0, short_circuit = False):
        '''
        Initializes application-related parameters:
        
        Args:
            speed_control (bool): 
                - True: Speed is controlled externally (e.g., by a dynamometer).
                - False: Speed is determined by torque and motor dynamics.
            commanded_speed (float): Final speed command [rad/sec].
            commanded_iq (float): Final commanded q-axis current [A].
            commanded_id (float): Final commanded d-axis current [A].
            acceleration (float): Acceleration [rad/sec^2]. Instantaneous if set to 0.
            current_ramp (float): Rate of change in current [A/sec].
            vbus (float): Supply voltage [V].
            init_speed (float): Initial speed [rad/sec].
            short_circuit (bool): 
                - True: Activates short circuit at a certain predetermined time.
                - False: Normal operation.
        '''
        self.speed_control = speed_control
        self.commanded_speed = commanded_speed
        self.commanded_iq = commanded_iq
        self.commanded_id = commanded_id
        self.acceleration = acceleration
        self.current_ramp = current_ramp
        self.vbus = vbus
        self.pi_v_lim = vbus * 0.75                 # Max allowed vq,vd outputs (Max allowed overmodulation).
        self.max_phase_v = vbus / 2                 # Max phase voltage
        self.init_speed = init_speed
        self.short_circuit = short_circuit

class MotorControl:
    def __init__(self, kp_d=0.2, ki_d=50.0, kp_q=0.2, ki_q=50.0, sampling_time=62.5e-6, dead_time = 300e-9):
        '''
        Initializes control related parameters:

        Args:
            Kp (n/a): current loop proportional gain
            Ki (n/a): current loop integral gain
            sampling_time (float): Time between artificial controller updates [sec]
            dead_time (float): Time window which both top and bottom transistors are off [sec]
        '''
        self.kp_d = kp_d
        self.ki_d = ki_d
        self.kp_q = kp_q
        self.ki_q = ki_q
        self.sampling_time = sampling_time
        self.half_sampling_time = self.sampling_time / 2
        self.integral_error_iq = 0
        self.integral_error_id = 0
        self.last_update_time = 0
        self.dead_time = dead_time
        self.saturation = 0
        self.mod_fact = 2 / np.sqrt(3)

    def pi_control(self, error_iq, error_id, current_time, vq, vd, pi_v_lim):
        """
        Parallel current loop PI controller.
        """          
        # Update voltages evey sampling time step
        if (current_time - self.last_update_time) >= self.sampling_time:
            self.integral_error_iq += error_iq * self.sampling_time * (1 - self.saturation)
            self.integral_error_id += error_id * self.sampling_time * (1 - self.saturation)            
            vq = self.kp_q * error_iq + self.ki_q * self.integral_error_iq
            vd = self.kp_d * error_id + self.ki_d * self.integral_error_id
            self.last_update_time = current_time
            # Saturation handling (Clamping)
            if ((vq**2 + vd**2) > pi_v_lim**2):
                # Clamp integrals
                self.saturation = 1
                # Prevent exceeding max vs
                volt_amp_gain = pi_v_lim / np.sqrt(vq**2 + vd**2)                
                vq *= volt_amp_gain
                vd *= volt_amp_gain
            else:
                self.saturation = 0
        else:
            return vq, vd
        
        return vq, vd

def inverse_dq_transform(q, d, angle):
    '''
    Inverse Direct DQ transformation
    '''
    a =  d * np.sin(angle) + q * np.cos(angle)
    b =  d * np.sin(angle - 2*np.pi/3) + q * np.cos(angle - 2*np.pi/3)
    c =  d * np.sin(angle + 2*np.pi/3) + q * np.cos(angle + 2*np.pi/3)
    return a, b, c

# Direct DQ transformation
def dq_transform(a, b, c, angle):
    '''
    Direct DQ transformation
    '''    
    d = (2/3) * (a * np.sin(angle) + b * np.sin(angle - 2*np.pi/3) + c * np.sin(angle + 2*np.pi/3))
    q = (2/3) * (a * np.cos(angle) + b * np.cos(angle - 2*np.pi/3) + c * np.cos(angle + 2*np.pi/3))    
    return q, d

def phase_current_ode(t, currents, va, vb, vc, motor):
    """
    Solve for current time derivatives. \n
    General phase voltage equation: \n
    v = i*R + L*di/dt + i*dL/dt + bemf * speed + Vn     ->       
    L * di/dt = (v - i*R - i*dL/dt - bemf - Vn) \n
    - v: terminal voltage (Voltage commanded by the drive unit). \n
    - i: phase current \n
    - R: phase resistance \n
    - L: induction \n
    - Vn: neutral voltage. For wye termination, phase voltage = v - Vn \n
    
    Solving as a linear matrix equation, Ax = b, where:
    
    - A = L (inductance matrix) concatenated with a row and a column of ones to add the KCL constraint and Vn to the equations \n
    - x = di/dt concatenated with Vn \n
    - b = v - i*R - i*dL/dt - bemf \n

    Args:
        va, vb, vc - terminal voltages (Voltage commanded by the drive unit).
        currents - phase currents
       
    Returns:
        current time derivatives (di/dt)
    """       

    ia, ib, ic = currents
    
    # A is the inductance matrix with neutral voltage handling allowing for Kirchhoff's Current Law (KCL) constraint (i_a + i_b + i_c = 0)
    A = np.array([
        [motor.Laa, motor.Lab, motor.Lac, 1],
        [motor.Lab, motor.Lbb, motor.Lbc, 1],
        [motor.Lac, motor.Lbc, motor.Lcc, 1],
        [1,         1,         1,         0]
        ])

    # The b vector (Terminal voltages minus resistive and flux terms)
    b = np.array([
        va - ia * motor.Rs - motor.bemf_a - motor.Laa_dot * ia - motor.Lab_dot * ib - motor.Lac_dot * ic,
        vb - ib * motor.Rs - motor.bemf_b - motor.Lab_dot * ia - motor.Lbb_dot * ib - motor.Lbc_dot * ic,
        vc - ic * motor.Rs - motor.bemf_c - motor.Lac_dot * ia - motor.Lbc_dot * ib - motor.Lcc_dot * ic,
        0   # KCL constraint i_a + i_b + i_c = 0
        ])

    # Solve for current derivatives and neutral voltage, Ax = b
    x = np.linalg.solve(A, b)

    di_a_dt, di_b_dt, di_c_dt, V_n = x

    return [di_a_dt, di_b_dt, di_c_dt]

def center_aligned_pwm_with_deadtime(va, vb, vc, max_v, t, pwm_period, half_period, dead_time):
    """
    Generates center-aligned PWM signals for top and bottom transistors with dead-time:

    Args:
        v (float): phase voltages [V]
        max_v (float): max phase voltage (vbus/2) [V]
        t (float): time in simulation [sec]
        pwm_period (float): equals to the sampling time [sec]
        dead_time (float): dead time [sec]

    Returns:
        array of pwm top and array of pwm bottom transistor values
    """    

    # Calculate the time in the current PWM period
    time_in_period = t % pwm_period

    # Calculate duty cycles for each phase (between 0 and 1, default is 0.5)
    duty_a = (va / max_v + 1) / 2
    duty_b = (vb / max_v + 1) / 2
    duty_c = (vc / max_v + 1) / 2

    # Find position along triangular carrier waveform
    carrier_wave = time_in_period / half_period if time_in_period < half_period else (pwm_period - time_in_period) / half_period

    # Generate the top and bottom PWM signals w/ dead time compensation (1 for high, 0 for low)
    pwm_a_top = 1 if carrier_wave > (1 - duty_a + (dead_time / 2) / half_period) else 0
    pwm_b_top = 1 if carrier_wave > (1 - duty_b + (dead_time / 2) / half_period) else 0
    pwm_c_top = 1 if carrier_wave > (1 - duty_c + (dead_time / 2) / half_period) else 0

    pwm_a_bottom = 1 if carrier_wave < (1 - duty_a - (dead_time / 2) / half_period) else 0
    pwm_b_bottom = 1 if carrier_wave < (1 - duty_b - (dead_time / 2) / half_period) else 0
    pwm_c_bottom = 1 if carrier_wave < (1 - duty_c - (dead_time / 2) / half_period) else 0

    return np.array([pwm_a_top, pwm_b_top, pwm_c_top]), np.array([pwm_a_bottom, pwm_b_bottom, pwm_c_bottom])


def terminal_voltage_with_deadtime(ia, ib, ic, pwm_signals_top, pwm_signals_bottom):
    """
    Set the terminal voltages while taking dead time into account based on current direction:

    Args:
        I (float): phase currents [A]
        pwm_signals (bool): transistor values (0 or 1)

    Returns:
        V_terminal (float): Terminal voltages [V]
    """
    # Initialize applied voltages
    va_terminal, vb_terminal, vc_terminal = 0, 0, 0

    # Phase A
    if pwm_signals_top[0] == 0 and pwm_signals_bottom[0] == 0:
        if ia > 0:
            va_terminal = 0  # Bottom transistor's voltage (ground)
        else:
            va_terminal = app.vbus  # Top transistor's voltage (bus voltage)
    else:
        va_terminal = pwm_signals_top[0] * app.vbus

    # Phase B
    if pwm_signals_top[1] == 0 and pwm_signals_bottom[1] == 0:
        if ib > 0:
            vb_terminal = 0  # Bottom transistor's voltage (ground)
        else:
            vb_terminal = app.vbus  # Top transistor's voltage (bus voltage)
    else:
        vb_terminal = pwm_signals_top[1] * app.vbus

    # Phase C
    if pwm_signals_top[2] == 0 and pwm_signals_bottom[2] == 0:
        if ic > 0:
            vc_terminal = 0  # Bottom transistor's voltage (ground)
        else:
            vc_terminal = app.vbus  # Top transistor's voltage (bus voltage)
    else:
        vc_terminal = pwm_signals_top[2] * app.vbus

    return va_terminal, vb_terminal, vc_terminal

def third_harmonic(va_in, vb_in, vc_in, mod_factor):
    """
    Add a third harmonic approximation (Not a true harmonic, but rather a triangular waveform)
    
    Args:
        v_in (float): phase voltages sinusoidally modulated [V]
        mod_factor (float): third harmonic factor 2/sqrt(3)

    Returns:
        v_out (float): phase voltages [V]
    """    
    va_out = va_in * mod_factor
    vb_out = vb_in * mod_factor
    vc_out = vc_in * mod_factor
    v_offset = -(max(va_out, vb_out, vc_out) + min(va_out, vb_out, vc_out)) / 2

    va_out += v_offset
    vb_out += v_offset
    vc_out += v_offset

    return va_out, vb_out, vc_out
    


def estimate_BW(control, app):
    '''
    Plots the bode plots of the close loop system response of the q and d axes.
    '''
    # Motor transfer function: G(s) = 1 / (L * s + r)
    num_d = [1]
    den_d = [motor.Ld, motor.Rs]
    num_q = [1]
    den_q = [motor.Lq, motor.Rs]
    # PI transfer function: G(s) = (Kp * s + Ki) / (s)
    num_pi_d = [control.kp_d, control.ki_d]
    den_pi_d = [1, 0]
    num_pi_q = [control.kp_q, control.ki_q]
    den_pi_q = [1, 0]    
    # Delay transfer function pade approxmation
    delay = control.sampling_time * 1.5
    num_delay, den_delay = ctrl.pade(delay, 5)

    # Create transfer functions
    G_d = ctrl.TransferFunction(num_d, den_d)
    PI_d = ctrl.TransferFunction(num_pi_d, den_pi_d)

    G_q = ctrl.TransferFunction(num_q, den_q)
    PI_q = ctrl.TransferFunction(num_pi_q, den_pi_q)

    G_delay = ctrl.TransferFunction(num_delay, den_delay)

    # If needed, voltage to PWM compare value.
    # For example, for a period value of 1000:
    # 50%   = 500   = 0 [V]
    # 0%    = 0     = -VBus/2
    # 100%  = 1000  = VBus/2
    # K_PWM = app.vbus / 1000
    K_PWM = 1

    OL_d = ctrl.series(G_d, PI_d, G_delay, K_PWM)
    CL_d = ctrl.feedback(OL_d,1)
    OL_q = ctrl.series(G_q, PI_q, G_delay, K_PWM)
    CL_q = ctrl.feedback(OL_q,1)

    # Plot Bode plots
    plt.figure(1)
    # Uncomment bode you wish to plot
    # ctrl.bode_plot(G_d, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'Plant', title = '')
    # ctrl.bode_plot(PI_d, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'PI', title = '')
    ctrl.bode_plot(OL_d, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'Open loop', title = '')
    ctrl.bode_plot(CL_d, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'Closed loop', title = '')

    # Finding first index where magnitude is lower than -3dB to find BW
    mag_d, _, omega_d = ctrl.frequency_response(CL_d, Hz=True, omega_limits=(10e0, 10e4))
    BWIndex = (np.where(mag_d < 0.707))[0][0]
    BW_d = omega_d[BWIndex] / (2 * np.pi)
    plt.title('D Axis:    BW = ' + "{:.0f}".format(BW_d) + '[Hz]', x = 0.5, y = 2.1, fontsize = 20)
    plt.legend()

    plt.figure(2)
    # Uncomment bode you wish to plot
    # ctrl.bode_plot(G_q, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'Plant', title = '')
    # ctrl.bode_plot(PI_q, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'PI', title = '')
    ctrl.bode_plot(OL_q, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'Open loop', title = '')
    ctrl.bode_plot(CL_q, dB=True, Hz=True, omega_limits=(10e0, 10e4), plot=True, label = 'Closed loop', title = '')

    # Finding first index where magnitude is lower than -3dB to find BW
    mag_q, _, omega_q = ctrl.frequency_response(CL_q, Hz=True, omega_limits=(10e0, 10e4))
    BWIndex = (np.where(mag_q < 0.707))[0][0]
    BW_q = omega_q[BWIndex] / (2 * np.pi)
    plt.title('Q Axis:    BW = ' + "{:.0f}".format(BW_q) + '[Hz]', x = 0.5, y = 2.1, fontsize = 20)
    plt.legend()

    plt.show()    

# Lists for plotting:
speed_list = []
iqd_ramped_list = []
iqd_sensed_list = []
error_list = []
vqd_list = []
vabc_sine_mod_list = []
vabc_list = []
pwm_list = []
v_terminal = []
bemf = []
currents = []    
torque_list = []
angle_list = []
dq_inductance_list = []
self_inductance_list = []
mutual_inductance_list = []
self_inductance_dot_list = []
mutual_inductance_dot_list = []
phase_volt_diff = []
phase_volt_diff_sine_mod = []

def simulate_motor(motor, sim, app, control):
    # Initializations
    speed_m = app.init_speed
    speed_e = speed_m * motor.pole_pairs
    angle_m = 0
    angle_e = 0
    iq_ramped = 0
    id_ramped = 0
    vq = 0
    vd = 0
    ia, ib, ic = 0, 0, 0
    torque = 0

    for t in tqdm(sim.time_points, desc="Running simulation", unit=" Cycles"):
        # Ramp handling
        # Speed ramp
        if app.speed_control:
            if (speed_m < app.commanded_speed) and (app.acceleration != 0):            
                speed_m += app.acceleration * sim.time_step
            else:
                speed_m = app.commanded_speed
        else:
            speed_m += ((torque - speed_m * motor.visc_fric_coeff) / motor.inertia) * sim.time_step
        speed_e = speed_m * motor.pole_pairs
        speed_list.append([speed_m, speed_e])

        # Current ramps
        if (abs(iq_ramped) < abs(app.commanded_iq)) and (app.current_ramp != 0):
            iq_ramped += app.current_ramp * sim.time_step * np.sign(app.commanded_iq)
        else:
            iq_ramped = app.commanded_iq

        if (abs(id_ramped) < abs(app.commanded_id)) and (app.current_ramp != 0):
            id_ramped += app.current_ramp * sim.time_step * np.sign(app.commanded_id)
        else:
            id_ramped = app.commanded_id        
        iqd_ramped_list.append([iq_ramped, id_ramped])

        # Convert abc frame currents to dq currents
        iq_sensed, id_sensed = dq_transform(ia, ib, ic, angle_e)
        iqd_sensed_list.append([iq_sensed, id_sensed])
        
        # Errors
        error_iq = iq_ramped - iq_sensed
        error_id = id_ramped - id_sensed    
        error_list.append([error_iq, error_id])
        
        # Calculate dq voltage commands
        vq, vd = control.pi_control(error_iq, error_id, t, vq, vd, app.pi_v_lim)
        vqd_list.append([vq, vd])
        
        # Convert Vdq voltages to abc frame
        va_sineMod_unclipped, vb_sineMod_unclipped, vc_sineMod_unclipped = inverse_dq_transform(vq, vd, angle_e)

        va_sineMod = max(min(va_sineMod_unclipped, app.max_phase_v), -app.max_phase_v)
        vb_sineMod = max(min(vb_sineMod_unclipped, app.max_phase_v), -app.max_phase_v)
        vc_sineMod = max(min(vc_sineMod_unclipped, app.max_phase_v), -app.max_phase_v)
        vabc_sine_mod_list.append([va_sineMod, vb_sineMod, vc_sineMod])
        
        # Add third harmonic approx. to sinusoidal modulation.
        va_unclipped, vb_unclipped, vc_unclipped = third_harmonic(va_sineMod_unclipped, vb_sineMod_unclipped, vc_sineMod_unclipped, control.mod_fact)
        va = max(min(va_unclipped, app.max_phase_v), -app.max_phase_v)
        vb = max(min(vb_unclipped, app.max_phase_v), -app.max_phase_v)
        vc = max(min(vc_unclipped, app.max_phase_v), -app.max_phase_v)        
        vabc_list.append([va, vb, vc])

        phase_volt_diff.append([va-vb, vb-vc, vc-va])
        phase_volt_diff_sine_mod.append([va_sineMod-vb_sineMod, vb_sineMod-vc_sineMod, vc_sineMod-va_sineMod])

        # Calculate transistor values including dead time        
        # Short circuit the phases at half the sim time (Arbitrary) if short_circuit == True
        if (app.short_circuit == False):# or ((app.short_circuit == True) and (t < (sim.total_time / 2))):
            pwm_signals_top, pwm_signals_bottom = center_aligned_pwm_with_deadtime(va, vb, vc, app.vbus/2, t, control.sampling_time, control.half_sampling_time, control.dead_time) 
        else:
            pwm_signals_top = [0, 0, 0]
            pwm_signals_bottom = [1, 1, 1]                    

        pwm_list.append([pwm_signals_top, pwm_signals_bottom])

        # Calculate terminal voltages including dead time (Terminal voltage are the voltages commanded by the drive unit, not the actual phase voltages.)
        va_terminal, vb_terminal, vc_terminal = terminal_voltage_with_deadtime(ia, ib, ic, pwm_signals_top, pwm_signals_bottom)
        v_terminal.append([va_terminal, vb_terminal, vc_terminal])

        # Update Ld, Lq
        motor.inductance_dq(iq_sensed, id_sensed)
        dq_inductance_list.append([motor.Lq, motor.Ld])
        
        # Update self and mutual phase inductances
        motor.inductance_abc(angle_e)
        self_inductance_list.append([motor.Laa, motor.Lbb, motor.Lcc])
        mutual_inductance_list.append([motor.Lab, motor.Lac, motor.Lbc])

        # Update self and mutual phase inductances time derivatives
        motor.inductance_abc_dot(angle_e, speed_e)
        self_inductance_dot_list.append([motor.Laa_dot, motor.Lbb_dot, motor.Lcc_dot])
        mutual_inductance_dot_list.append([motor.Lab_dot, motor.Lac_dot, motor.Lbc_dot])

        # Calculate the phases bemf
        motor.bemf_a = speed_m * motor.phase_bemf(angle_e, 0)
        motor.bemf_b = speed_m * motor.phase_bemf(angle_e, -2 * np.pi / 3)
        motor.bemf_c = speed_m * motor.phase_bemf(angle_e, 2 * np.pi / 3)
        bemf.append([motor.bemf_a, motor.bemf_b, motor.bemf_c])

        # Solve the ODE for phase currents over one time step
        sol = solve_ivp(phase_current_ode, [t, t + sim.time_step], [ia, ib, ic],
                        args=(va_terminal, vb_terminal, vc_terminal, motor), method='RK45')    

        ia, ib, ic = sol.y[:, -1]
        currents.append([ia, ib, ic])        

        torque = motor.torque(iq_sensed, id_sensed)
        torque_list.append(torque)

        angle_m += speed_m * sim.time_step
        angle_e += speed_e * sim.time_step
        angle_list.append([angle_m, angle_e])

# Instantiate objects
motor = Motor()
sim = Simulation()
app = Application()
control = MotorControl()

# Uncomment to show closed loop bode plots of q and d axes:
# estimate_BW(control, app)

# Run the simulation
simulate_motor(motor, sim, app, control)

# Plot results
time_points = sim.time_points
speed_list = np.array(speed_list)
iqd_ramped_list = np.array(iqd_ramped_list)
iqd_sensed_list = np.array(iqd_sensed_list)
error_list = np.array(error_list)
vqd_list = np.array(vqd_list)
vabc_sine_mod_list = np.array(vabc_sine_mod_list)
vabc_list = np.array(vabc_list)
pwm_list = np.array(pwm_list)
v_terminal = np.array(v_terminal)
bemf = np.array(bemf)
currents = np.array(currents)
torque = np.array(torque_list)
angle_list = np.array(angle_list)
dq_inductance_list = np.array(dq_inductance_list)
self_inductance_list = np.array(self_inductance_list)
mutual_inductance_list = np.array(mutual_inductance_list)
self_inductance_dot_list = np.array(self_inductance_dot_list)
mutual_inductance_dot_list = np.array(mutual_inductance_dot_list)
phase_volt_diff = np.array(phase_volt_diff)
phase_volt_diff_sine_mod = np.array(phase_volt_diff_sine_mod)
voltage_amplitude = np.sqrt(vqd_list[:, 0]**2 + vqd_list[:, 1]**2)
voltage_limit = np.ones_like(time_points) * app.vbus / np.sqrt(3)

plt.figure(figsize=(10, 8))

plt.subplot(4, 1, 1)
plt.plot(time_points, iqd_sensed_list[:, 0], label='iqSensed')
plt.plot(time_points, iqd_sensed_list[:, 1], label='idSensed')
plt.plot(time_points, iqd_ramped_list[:, 0], label='iqCmd')
plt.plot(time_points, iqd_ramped_list[:, 1], label='idCmd')
plt.title('iq, Id Cmd + Sensed')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time_points, vqd_list[:, 0], label='Vq')
plt.plot(time_points, vqd_list[:, 1], label='Vd')
plt.plot(time_points, voltage_amplitude, label='Voltage amplitude')
plt.plot(time_points, voltage_limit, label='limit', linestyle = 'dashed', color = 'red')
plt.title('Vqd')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time_points, currents[:, 0], label='Ia')
plt.plot(time_points, currents[:, 1], label='Ib')
plt.plot(time_points, currents[:, 2], label='Ic')
plt.title('Currents')
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time_points, bemf[:, 0], label='bemf_a')
plt.plot(time_points, bemf[:, 1], label='bemf_b')
plt.plot(time_points, bemf[:, 2], label='bemf_c')
plt.title('Back-emf')
plt.legend()

plt.tight_layout()
plt.show()


'''
TODO:
Add support for unbalanced phases (different inductance/resistance per phase).
Add support for induction motors.
'''
