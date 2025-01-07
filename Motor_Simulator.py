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

class Config:
    def __init__(self):
        '''
        Initializes all script parameters:

        Args:
            # Motor parameters
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
            harmonics (dict): BEMF harmonics.

            # Simulation parameters
            time_step (float): Simualtion time step [sec]
            total_time (float): Total simulation time [sec]            

            # Application parameters
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

            # Control parameters
            Kp (n/a): current loop proportional gain
            Ki (n/a): current loop integral gain
            pwm_period (n/a): center aligned PWM period
            k_shifting (n/a): additional gain, e.g to counteract PI shifting.
            sampling_time (float): Time between artificial controller updates [sec]
            dead_time (float): Time window which both top and bottom transistors are off [sec]                
            afc_ki_q (n/a): AFC q axis integral gain
            afc_ki_d (n/a): AFC d axis integral gain
            afc_harmonic (int): Harmonic in the dq axis
            afc_method (int):
                - 0: Inactive
                - 1: Attenuates harmonic oscillations on dq currents.
                - 2: Attenuates harmonic oscillations on dq voltages.

        '''
        # Motor parameters
        self.motor_type = "SYNC"
        self.pole_pairs = 4
        self.Rs = 0.0028
        self.Lq_base = 0.000077
        self.Ld_base = 0.0000458
        self.bemf_const = 0.11459
        self.inertia = 0.01
        self.visc_fric_coeff = 0.005
        self.i_max = 600
        # Harmonics, choose preferred option (Comment out the other):
        #   None for no bemf harmonics
        #   dictionary for desired harmonics
        self.harmonics = None
        # self.harmonics = {1: {'harmonic': 5, 'mag': -self.bemf_const / 20},
        #                   2: {'harmonic': 7, 'mag': self.bemf_const / 20},
        #                   3: {'harmonic': 11, 'mag': -self.bemf_const / 40},
        #                   4: {'harmonic': 13, 'mag': self.bemf_const / 40}}
        
        # Simulation parameters
        self.time_step = 2000e-9
        self.total_time = 0.01

        # Application parameters
        self.speed_control = True
        self.commanded_speed = 100.0
        self.commanded_iq = 200.0
        self.commanded_id = -50
        self.acceleration = 10000.0
        self.current_ramp = 10000.0
        self.vbus = 48.0
        self.init_speed = 0.0
        self.short_circuit = False

        # Control parameters
        self.kp_d = 0.2
        self.ki_d = 50.0
        self.kp_q = 0.2
        self.ki_q = 50.0
        self.pwm_period = 1500
        # To be able to use the same PI control parameters as used in a practical controller
        # Additional gains must be used to:
            # Output voltages and not compare values (k_pwm).
            # Taking into account shifting that occurs in microprocessors to avoid floating points (k_shifting).
        self.k_pwm = (self.vbus / 2) / self.pwm_period          # max phase voltage / PWM period
        self.k_shifting = 2**16
        self.sampling_time = 62.5e-6
        self.dead_time = 300e-9
        self.afc_ki_q = 0.05
        self.afc_ki_d = 0.05
        self.afc_harmonic = 6
        self.afc_method = 0

class Motor:
    def __init__(self, config):
        '''
        Specifies motor-related parameters.
        '''

        self.motor_type = config.motor_type
        self.pole_pairs = config.pole_pairs
        self.Rs = config.Rs
        self.Lq_base = config.Lq_base
        self.Ld_base = config.Ld_base
        self.Lq = config.Lq_base
        self.Ld = config.Ld_base        
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
        self.bemf_const = config.bemf_const / np.sqrt(3)       # Convert from line-to-line to phase (Wye topology)
        self.bemf_a = 0
        self.bemf_b = 0
        self.bemf_c = 0        
        self.flux_linkage = config.bemf_const / config.pole_pairs / 1.5
        self.harmonics = config.harmonics
        self.inertia = config.inertia
        self.visc_fric_coeff = config.visc_fric_coeff
        self.i_max = config.i_max

        

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
        Calculate bemf, allow for harmonics:

        Args:
            angle (float): electrical angle [rad].
            phase_shift (float): phase shift between the phases [rad].
        
        Returns:
            bemf (float): bemf of phase at current angle [V]
        """          
        bemf = self.bemf_const * np.cos(angle + phase_shift)
        if self.harmonics:
            for _, data in self.harmonics.items():
                bemf += data['mag'] * np.cos(data['harmonic'] * (angle + phase_shift))
        return bemf

    def torque(self, iq, id):
        """
        Calculate Torque based on motor type: Synchronous or asynchronous:

        Args:
            iq (float): iq current [A].
            id (float): id current [A].
        
        Returns:
            torque (float): calculated torque [Nm]
        """           
        if self.motor_type == "SYNC":
            torque = 1.5 * self.pole_pairs * (self.flux_linkage * iq + (self.Ld - self.Lq) * iq * id)
        else: # ASYNC motor
            torque = 1.5 * self.pole_pairs * (self.flux_linkage * iq + (self.Ld - self.Lq) * iq * id) # TODO: Update equation for async motors
        return torque

class Simulation:
    def __init__(self, config):
        '''
        Initializes simulation related parameters:
        '''
        self.time_step = config.time_step
        self.total_time = config.total_time
        self.time_points = np.arange(0, config.total_time, config.time_step)

class Application:
    def __init__(self, config):
        '''
        Initializes application-related parameters:
        '''
        self.speed_control = config.speed_control
        self.commanded_speed = config.commanded_speed
        self.commanded_iq = config.commanded_iq
        self.commanded_id = config.commanded_id
        self.acceleration = config.acceleration
        self.current_ramp = config.current_ramp
        self.vbus = config.vbus
        self.pi_v_lim = config.vbus * 0.75                 # Max allowed vq,vd outputs (Max allowed overmodulation).
        self.max_phase_v = config.vbus / 2                 # Max phase voltage
        self.init_speed = config.init_speed
        self.short_circuit = config.short_circuit

class MotorControl:
    def __init__(self, config):
        '''
        Initializes control related parameters:
        '''
        self.kp_d = config.k_pwm * config.kp_d / config.k_shifting
        self.ki_d = config.k_pwm * config.ki_d / config.k_shifting
        self.kp_q = config.k_pwm * config.kp_q / config.k_shifting
        self.ki_q = config.k_pwm * config.ki_q / config.k_shifting
        self.vd = 0
        self.vq = 0
        self.sampling_time = config.sampling_time
        self.half_sampling_time = config.sampling_time / 2
        self.integral_error_iq = 0
        self.integral_error_id = 0
        self.last_update_time = 0
        self.dead_time = config.dead_time
        self.saturation = 0
        self.mod_fact = 2 / np.sqrt(3)
        self.afc_ki_d = config.afc_ki_d
        self.afc_ki_q = config.afc_ki_q
        self.afc_sin_integral_error_d = 0
        self.afc_sin_integral_error_q = 0        
        self.afc_cos_integral_error_d = 0
        self.afc_cos_integral_error_q = 0        
        self.afc_harmonic = config.afc_harmonic
        self.afc_method = config.afc_method
        self.afc_id = 0
        self.afc_iq = 0
        self.afc_vd = 0
        self.afc_vq = 0

    def pi_control(self, error_iq, error_id):
        """
        Parallel current loop PI controller:
        
        Args:
            error_iq (float): iq error (ref - sensed) [A].
            error_id (float): id error (ref - sensed) [A].
        """
        # Update voltages evey sampling time step
        self.integral_error_iq += error_iq * self.sampling_time * (1 - self.saturation)
        self.integral_error_id += error_id * self.sampling_time * (1 - self.saturation)            
        self.vq = self.kp_q * error_iq + self.ki_q * self.integral_error_iq
        self.vd = self.kp_d * error_id + self.ki_d * self.integral_error_id        
    
    def afc_control(self, error_iq, error_id, angle):    
        '''
        Adaptive feedforward cancellation, attenuates harmonic oscillations on dq voltages or currents, based on method:

        Args:
            error_iq (float): iq error (ref - sensed) [A].
            error_id (float): id error (ref - sensed) [A].
            angle (float): electrical angle [rad].
        '''
        if self.afc_method > 0:
            harmonic_angle = self.afc_harmonic * angle
            harmonic_sin = np.sin(harmonic_angle)
            harmonic_cos = np.cos(harmonic_angle)
            err_post_gain_id = self.afc_ki_d * error_id
            err_post_gain_iq = self.afc_ki_q * error_iq
            
            self.afc_sin_integral_error_d += harmonic_sin * (err_post_gain_id)
            self.afc_sin_integral_error_q += harmonic_sin * (err_post_gain_iq)
            self.afc_cos_integral_error_d += harmonic_cos * (err_post_gain_id)
            self.afc_cos_integral_error_q += harmonic_cos * (err_post_gain_iq)

            if self.afc_method == 1:
                self.afc_vd = self.afc_sin_integral_error_d * harmonic_sin + self.afc_cos_integral_error_d * harmonic_cos
                self.afc_vq = self.afc_sin_integral_error_q * harmonic_sin + self.afc_cos_integral_error_q * harmonic_cos                
                self.afc_id = 0
                self.afc_iq = 0
            else:
                self.afc_id = self.afc_sin_integral_error_d * harmonic_sin + self.afc_cos_integral_error_d * harmonic_cos
                self.afc_iq = self.afc_sin_integral_error_q * harmonic_sin + self.afc_cos_integral_error_q * harmonic_cos
                self.afc_vd = 0
                self.afc_vq = 0

    def voltage_limiter(self, pi_v_lim):
        '''
        Voltage limiter:

        Args:
            pi_v_lim (float): Max allowed vq,vd outputs (Max allowed overmodulation) [V].
        '''        
        v_amp_sqr = self.vq**2 + self.vd**2
        # Saturation handling (Clamping)
        if (v_amp_sqr > pi_v_lim**2):
            # Clamp integrals
            self.saturation = 1
            # Prevent exceeding max vs
            volt_amp_gain = pi_v_lim / np.sqrt(v_amp_sqr)                
            self.vq *= volt_amp_gain
            self.vd *= volt_amp_gain
        else:
            self.saturation = 0        



def inverse_dq_transform(q, d, angle):
    '''
    Inverse Direct DQ transformation:

    Args:
        q (float): q value [A or V].
        d (float): d value [A or V].
        angle (float): electrical angle [rad].

    Returns:
        a,b,c (float): phase voltages or currents [A or V]
    '''
    a =  d * np.sin(angle) + q * np.cos(angle)
    b =  d * np.sin(angle - 2*np.pi/3) + q * np.cos(angle - 2*np.pi/3)
    c =  d * np.sin(angle + 2*np.pi/3) + q * np.cos(angle + 2*np.pi/3)
    return a, b, c

# Direct DQ transformation
def dq_transform(a, b, c, angle):
    '''
    Direct DQ transformation:

    Args:
        a (float): a value [A or V].
        b (float): b value [A or V].
        c (float): c value [A or V].
        angle (float): electrical angle [rad].

    Returns:
        q,d (float): dq currents or voltages [A or V]        
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

    OL_d = ctrl.series(G_d, PI_d, G_delay)
    CL_d = ctrl.feedback(OL_d,1)
    OL_q = ctrl.series(G_q, PI_q, G_delay)
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
afc_integrals = []
afc_outputs = []

def simulate_motor(motor, sim, app, control):
    # Initializations
    speed_m = app.init_speed
    speed_e = speed_m * motor.pole_pairs
    angle_m = 0
    angle_e = 0
    iq_ramped = 0
    id_ramped = 0
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
        error_iq = iq_ramped - iq_sensed - control.afc_iq
        error_id = id_ramped - id_sensed - control.afc_id
        error_list.append([error_iq, error_id])

        # Run control loops every sampling time step
        if (t - control.last_update_time) >= control.sampling_time:
            # Calculate dq voltage commands
            control.pi_control(error_iq, error_id)

            # AFC calculations
            control.afc_control(error_iq, error_id, angle_e)        

            # Voltage limiter
            control.voltage_limiter(app.pi_v_lim)

            control.vq += control.afc_vq
            control.vd += control.afc_vd
            control.last_update_time = t      
                    
        vqd_list.append([control.vq, control.vd])
        afc_integrals.append([control.afc_sin_integral_error_d, control.afc_sin_integral_error_q, control.afc_cos_integral_error_d, control.afc_cos_integral_error_q])
        afc_outputs.append([control.afc_vq, control.afc_vd, control.afc_iq, control.afc_id])        
        
        # Convert Vdq voltages to abc frame
        va_sineMod_unclipped, vb_sineMod_unclipped, vc_sineMod_unclipped = inverse_dq_transform(control.vq, control.vd, angle_e)

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
config = Config()
motor = Motor(config)
sim = Simulation(config)
app = Application(config)
control = MotorControl(config)

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
afc_integrals = np.array(afc_integrals)
afc_outputs = np.array(afc_outputs)

# Plotting data dictionary. Contains all available variables to plot.
data = {
    "speed_m:": speed_list[:,0],
    "speed_e:": speed_list[:,1],
    "iqCmd": iqd_ramped_list[:, 0],
    "idCmd": iqd_ramped_list[:, 1],    
    "iqSensed": iqd_sensed_list[:, 0],
    "idSensed": iqd_sensed_list[:, 1],
    "err_iq": error_list[:, 0],
    "err_id": error_list[:, 1],    
    "vq": vqd_list[:, 0],
    "vd": vqd_list[:, 1],        
    "va_sin": vabc_sine_mod_list[:, 0],
    "vb_sin": vabc_sine_mod_list[:, 1],
    "vc_sin": vabc_sine_mod_list[:, 2],    
    "va": vabc_list[:, 0],        
    "vb": vabc_list[:, 1],
    "vc": vabc_list[:, 2],    
    "pwmTopA": pwm_list[:, 0, 0],
    "pwmTopB": pwm_list[:, 0, 1],
    "pwmTopC": pwm_list[:, 0, 2],
    "pwmBottomA": pwm_list[:, 1, 0],
    "pwmBottomB": pwm_list[:, 1, 1],
    "pwmBottomC": pwm_list[:, 1, 2],
    "va_terminal": v_terminal[:, 0],
    "vb_terminal": v_terminal[:, 1],    
    "vc_terminal": v_terminal[:, 2],        
    "bemf_a": bemf[:, 0],
    "bemf_b": bemf[:, 1],    
    "bemf_c": bemf[:, 2],        
    "ia": currents[:, 0],
    "ib": currents[:, 1],    
    "ic": currents[:, 2],
    "torque": torque,
    "angle_m": angle_list[:, 0],
    "angle_e": angle_list[:, 1],    
    "Lq": dq_inductance_list[:, 0],
    "Ld": dq_inductance_list[:, 1],
    "La": self_inductance_list[:, 0],
    "Lb": self_inductance_list[:, 1],    
    "Lc": self_inductance_list[:, 2],
    "Lab": mutual_inductance_list[:, 0],
    "Lac": mutual_inductance_list[:, 1],    
    "Lbc": mutual_inductance_list[:, 2],
    "La_dot": self_inductance_dot_list[:, 0],
    "Lb_dot": self_inductance_dot_list[:, 1],    
    "Lc_dot": self_inductance_dot_list[:, 2],    
    "Lab_dot": mutual_inductance_dot_list[:, 0],
    "Lac_dot": mutual_inductance_dot_list[:, 1],    
    "Lbc_dot": mutual_inductance_dot_list[:, 2],
    "va-vb": phase_volt_diff[:, 0],
    "vb-vc": phase_volt_diff[:, 1],    
    "vc-va": phase_volt_diff[:, 2],
    "va_sineMod-vb_sineMod": phase_volt_diff_sine_mod[:, 0],
    "vb_sineMod-vc_sineMod": phase_volt_diff_sine_mod[:, 1],    
    "vc_sineMod-va_sineMod": phase_volt_diff_sine_mod[:, 2],
    "v_lim": voltage_limit,
    "v_amp": voltage_amplitude,
    "afc_sin_d": afc_integrals[:, 0],
    "afc_sin_q": afc_integrals[:, 1],
    "afc_cos_d": afc_integrals[:, 2],
    "afc_cos_q": afc_integrals[:, 3],
    "afc_vq": afc_outputs[:, 0],    
    "afc_vd": afc_outputs[:, 1],
    "afc_iq": afc_outputs[:, 2],
    "afc_id": afc_outputs[:, 3],
}


# Control which plot appears in which subplot (1 = appears in subplot 1, 2 = appears in subplot 2, etc.)
# If it doesn't appear it won't be plotted
plot_options = {
    "iqSensed": 1,
    "idSensed": 1,
    "iqCmd": 1,
    "idCmd": 1,
    "vq": 2,
    "vd": 2,
    "v_amp": 2,
    "v_lim": 2,
    "ia": 3,
    "ib": 3,
    "ic": 3,
    "bemf_a": 4,
    "bemf_b": 4,
    "bemf_c": 4,    
}


# Group plots by subplot number
grouped_plots = {}
for label, subplot_num in plot_options.items():
    if subplot_num > 0:  # Ignore 0 (don't plot)
        if subplot_num not in grouped_plots:
            grouped_plots[subplot_num] = []  # Create new group
        grouped_plots[subplot_num].append(label)

# Create subplots dynamically
num_subplots = max(grouped_plots.keys())  # Determine total subplots needed
plt.figure(figsize=(10, 3 * num_subplots))  # Adjust size based on number of subplots

for i in range(1, num_subplots + 1):
    plt.subplot(num_subplots, 1, i)
    for label in grouped_plots[i]:
        plt.plot(time_points, data[label], label=label)
    plt.title(", ".join(grouped_plots[i]))  # Title based on plot names
    plt.grid(True)
    plt.legend()

# Adjust layout and show
plt.tight_layout()
plt.show()


'''
TODO:
Add support for unbalanced phases (different inductance/resistance per phase).
Add support for induction motors.
'''
