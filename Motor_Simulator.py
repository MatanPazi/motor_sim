"""
Script Name: Motor_Simulator.py
Author: Matan Pazi
Date: January 27th, 2025
Version: 3.3
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
from scipy.optimize import minimize
from math import sin, cos, radians, sqrt
import tkinter as tk
from tkinter import filedialog, scrolledtext
import re

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
            x_factor (float): The change in phase impedance from the nominal values.
                            If all x_factors are equal (= 1), the motor phases are balanced.
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
            generate_lut (bool): 
                - True: Auto generate an MTPA lut.
                - False: Load a text file with iq and id look up tables.            
            speed_control (bool): 
                - True: Speed is controlled externally (e.g., by a dynamometer).
                - False: Speed is determined by torque and motor dynamics.
            commanded_speed (float): Final speed command [rad/sec].
            torque_command_flag (bool): Decides whether to command torque using a lookup table generated
                                        based on motor parameters or commanding iq, id directly
                - True: Command torque using a LUT.
                - False: Command iq, id directly.
            commanded_iq (float): Final commanded q-axis current [A].
            commanded_id (float): Final commanded d-axis current [A].
            commanded_torque (float): Final commanded torque command [Nm].
            acceleration (float): Acceleration [rad/sec^2]. Instantaneous if set to 0.
            current_ramp (float): Rate of change in current [A/sec].
            torque_ramp (float): Rate of change in torque [Nm/sec].
            v_batt_init (float): Initial battery voltage [V].
            init_speed (float): Initial speed [rad/sec].
            short_circuit (bool):
                - True: Activates short circuit at a certain predetermined time.
                - False: Normal operation.
            battery_capacity (float): Battery capacity [Ah].
            battery_max_voltage (float): Battery max voltage [V]
            battery_resistance (float): Battery internal resistance [Ohm].
                                                Taken as an estimate from the discharge curves. Includes the total # of cells in parallel and series.
            battery_inductance (float): Battery internal inductance [H]
            cable_resistance (float): Cable connecting battery and inverter resistance [Ohm].
            cable_inductance (float): Cable connecting battery and inverter inductance [H]            
            dc_link_capacitance (float): DC link capacitors total capacitance [F]
            dc_link_resistance (float): DC link capacitors ESR [Ohm]            
            transistor_resistance (float): Average high/low transistor ON resistance (Ohm)
            switch_energy_loss (float): Transistor energy loss per switch [Joule]


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
            decoupling_enabled (bool):
                - 0: Off   
                - 1: On
            mod_speed_threshold (float): Voltage amplitude above which modified speed protection takes effect [V]
            mod_speed_kp (n/a): modified speed proportional gain
            mod_speed_ki (n/a): modified speed integral gain

            # MTPA lookup table generator parameters
            torque_max (float): Maximum torque at maximum current [Nm].
            speed_max (float): Maximum speed [rad/sec].    
            torque_increment (int): torque increment size in the mtpa table [Nm].
            speed_increment (int): speed increment size in the mtpa table [rad/sec].

            # Low pass filter parameters
            phase_current_cutoff_freq (float): filter cutoff frequency

        '''
        # Motor parameters
        self.motor_type = "SYNC"
        self.pole_pairs = 4
        self.Rs = 0.0031
        self.Lq_base = 0.000101
        self.Ld_base = 0.000067
        self.a_factor = 1
        self.b_factor = 1
        self.c_factor = 1
        self.bemf_const = 0.102
        self.inertia = 0.01
        self.visc_fric_coeff = 0.005
        self.i_max = 750
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
        self.total_time = 0.05

        # Application parameters
        self.generate_lut = True
        self.speed_control = True
        self.commanded_speed = 1000 * (2 * np.pi / 60)
        self.torque_command_flag = False    # If you wish to command torque using a lut, set to True
        self.commanded_iq = 200
        self.commanded_id = -50
        self.commanded_torque = 10
        self.acceleration = 10000
        self.current_ramp = 10000
        self.torque_ramp = 1000
        self.v_batt_init = 44.0
        self.init_speed = 0.0
        self.short_circuit = False
        self.battery_capacity = 40
        self.battery_max_voltage = 58.8
        self.battery_resistance = 0.015
        self.battery_inductance = 0.000003
        self.cable_resistance = 0.002
        self.cable_inductance = 0.000005
        self.dc_link_capacitance = 0.004
        self.dc_link_resistance = 0.002
        self.transistor_resistance = 0.001
        self.switch_energy_loss = 0.001

        # Control parameters
        self.kp_d = 70000
        self.ki_d = 450
        self.kp_q = 75000
        self.ki_q = 400
        self.pwm_period = 1024
        # To be able to use the same PI control parameters as used in a practical controller
        # Additional gains must be used to:
            # Output voltages and not compare values (k_pwm).
            # Taking into account shifting that occurs in microprocessors to avoid floating points (k_shifting).
        self.k_pwm = (self.v_batt_init / 2) / self.pwm_period          # max phase voltage / PWM period
        self.k_shifting = 2**13
        self.sampling_time = 62.5e-6
        self.dead_time = 0e-9
        self.afc_ki_q = 0.05
        self.afc_ki_d = 0.05
        self.afc_harmonic = 6
        self.afc_method = 0
        self.decoupling_enabled = 0
        self.mod_speed_threshold = self.v_batt_init / np.sqrt(3)     # see "A Quick Look on Three-phase Overmodulation Waveforms"
        self.mod_speed_kp = 20000
        self.mod_speed_ki = 5000

        # MTPA lookup table generator parameters
        self.torque_max = 72
        self.speed_max = 7000 * (2 * np.pi / 60)
        self.torque_increment = 3
        self.speed_increment = 350 * (2 * np.pi / 60)

        # Low pass filter parameters
        self.phase_current_cutoff_freq = 5000

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
        self.a_factor = config.a_factor
        self.b_factor = config.b_factor
        self.c_factor = config.c_factor
        self.Ra = self.a_factor * self.Rs
        self.Rb = self.b_factor * self.Rs
        self.Rc = self.c_factor * self.Rs
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
        self.flux_linkage = config.bemf_const / config.pole_pairs / np.sqrt(2)
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
            self.Lq = self.Lq_base * (1 - 0.4 * Is/self.i_max)
            self.Ld = self.Ld_base * (1 - 0.3 * Is/self.i_max)
        else:
            self.Lq = self.Lq_base * 0.6
            self.Ld = self.Ld_base * 0.7
    
    def inductance_abc(self, theta):
        """
        Update the inductances in the abc frame (Based on the dq transform)
        """        
        self.Laa = self.a_factor * self.Lq * (np.cos(theta))**2 + self.Ld * (np.sin(theta))**2
        self.Lbb = self.b_factor * self.Lq * (np.cos(theta - 2*np.pi/3))**2 + self.Ld * (np.sin(theta - 2*np.pi/3))**2
        self.Lcc = self.c_factor * self.Lq * (np.cos(theta + 2*np.pi/3))**2 + self.Ld * (np.sin(theta + 2*np.pi/3))**2   

        # Mutual inductance - Assuming position dependency only.
        self.Lab = np.sqrt(self.a_factor * self.b_factor) * self.Lq * np.cos(theta) * np.cos(theta - 2*np.pi/3) + self.Ld * np.sin(theta) * np.sin(theta - 2*np.pi/3)
        self.Lac = np.sqrt(self.a_factor * self.c_factor) * self.Lq * np.cos(theta) * np.cos(theta + 2*np.pi/3) + self.Ld * np.sin(theta) * np.sin(theta + 2*np.pi/3)
        self.Lbc = np.sqrt(self.b_factor * self.c_factor) * self.Lq * np.cos(theta - 2*np.pi/3) * np.cos(theta + 2*np.pi/3) + self.Ld * np.sin(theta - 2*np.pi/3) * np.sin(theta + 2*np.pi/3)

    def inductance_abc_dot(self, theta, speed):        
        """
        Update the inductances time derivatives
        """  
        # Derivatives for self inductances
        self.Laa_dot = -self.a_factor * (self.Lq - self.Ld) * np.sin(2 * theta) * speed
        self.Lbb_dot = -self.b_factor * (self.Lq - self.Ld) * np.cos(2 * theta + np.pi/6) * speed
        self.Lcc_dot = self.c_factor * (self.Lq - self.Ld) * np.sin(2 * (theta + np.pi/6)) * speed
        
        # Derivatives for mutual inductances
        self.Lab_dot = np.sqrt(self.a_factor * self.b_factor) * (self.Lq - self.Ld) * np.cos(np.pi/6 - 2 * theta) * speed
        self.Lac_dot = -np.sqrt(self.a_factor * self.c_factor) * (self.Lq - self.Ld) * np.cos(2 * theta + np.pi/6) * speed
        self.Lbc_dot = -np.sqrt(self.b_factor * self.c_factor) * (self.Lq - self.Ld) * np.sin(2 * theta) * speed

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
        self.generate_lut = config.generate_lut
        self.speed_control = config.speed_control
        self.commanded_speed = config.commanded_speed
        self.torque_command_flag = config.torque_command_flag
        self.commanded_iq = config.commanded_iq
        self.commanded_id = config.commanded_id
        self.commanded_torque = config.commanded_torque
        self.acceleration = config.acceleration
        self.current_ramp = config.current_ramp
        self.torque_ramp = config.torque_ramp
        self.v_batt_init = config.v_batt_init
        self.v_bus = self.v_batt_init
        self.init_speed = config.init_speed
        self.short_circuit = config.short_circuit
        self.battery_capacity = config.battery_capacity * 3600                          # Converting to coulomb
        self.battery_max_voltage = config.battery_max_voltage
        self.battery_capacitance = self.battery_capacity / self.battery_max_voltage     # Conservative estimate of capacitance [Farad]
        self.battery_resistance = config.battery_resistance

class MotorControl:
    def __init__(self, config):
        '''
        Initializes control related parameters:
        '''
        self.kp_d = config.k_pwm * config.kp_d / config.k_shifting
        self.ki_d = config.k_pwm * config.ki_d / config.k_shifting / config.sampling_time
        self.kp_q = config.k_pwm * config.kp_q / config.k_shifting
        self.ki_q = config.k_pwm * config.ki_q / config.k_shifting / config.sampling_time
        self.vd = 0
        self.vq = 0
        self.vs = 0
        self.pi_v_lim = config.v_batt_init * 0.65                 # Slightly above 2/pi, which is max overmodulation, see "A Quick Look on Three-phase Overmodulation Waveforms"
        self.max_phase_v = config.v_batt_init / 2                 # Max phase voltage
        self.sampling_time = config.sampling_time
        self.sampling_frequency = 1 / self.sampling_time
        self.half_sampling_time = config.sampling_time / 2
        self.pi_integral_out_q = 0
        self.pi_integral_out_d = 0
        self.pi_proportional_out_q = 0
        self.pi_proportional_out_d = 0
        self.pi_vq = 0                  # [V]
        self.pi_vd = 0                  # [V]        
        self.last_update_time = 0
        self.dead_time = config.dead_time
        self.mod_fact = 2 / np.sqrt(3)
        self.afc_ki_d = config.k_pwm * config.afc_ki_d / config.k_shifting
        self.afc_ki_q = config.k_pwm * config.afc_ki_q / config.k_shifting
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
        self.decoupling_vq = 0
        self.decoupling_vd = 0
        self.decoupling_enabled = config.decoupling_enabled
        self.mod_speed_threshold = config.mod_speed_threshold
        self.mod_speed_kp = config.mod_speed_kp / config.k_shifting
        self.mod_speed_ki = config.mod_speed_ki / config.k_shifting / config.sampling_time
        self.mod_speed_integral_out = 0
        self.mod_speed_proportional_out = 0
        self.mod_speed_out = 0          # [rad/sec]


    def pi_control(self, error_iq, error_id):
        """
        Parallel current loop PI controller:
        
        Args:
            error_iq (float): iq error (ref - sensed) [A].
            error_id (float): id error (ref - sensed) [A].
        """
        # Update voltages evey sampling time step
        self.pi_integral_out_q += self.ki_q * error_iq * self.sampling_time
        self.pi_integral_out_d += self.ki_d * error_id * self.sampling_time

        # Clamping
        if abs(self.pi_integral_out_q) > self.pi_v_lim:
            self.pi_integral_out_q = self.pi_v_lim * np.sign(self.pi_integral_out_q)
        if abs(self.pi_integral_out_d) > self.pi_v_lim:
            self.pi_integral_out_d = self.pi_v_lim * np.sign(self.pi_integral_out_d)

        self.pi_proportional_out_q = self.kp_q * error_iq
        self.pi_proportional_out_d = self.kp_d * error_id
        self.pi_vq = self.pi_proportional_out_q + self.pi_integral_out_q        
        self.pi_vd = self.pi_proportional_out_d + self.pi_integral_out_d
        
        # Limiting each axis independently, though limiting the vs amplitude would be more correct..
        if abs(self.pi_vq) > self.pi_v_lim:
            self.pi_vq = self.pi_v_lim * np.sign(self.pi_vq)
        if abs(self.pi_vd) > self.pi_v_lim:
            self.pi_vd = self.pi_v_lim * np.sign(self.pi_vd)
    
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

    def decoupling(self, iq, id, speed, bemf_const, Ld, Lq):    
        '''
        Adding feedforward voltages to vd and vq to remove any addition not relating to id and iq respectively:        

        Args:
            iq (float): iq ref command [A].
            id (float): id ref command [A].
            speed (float): electrical speed [rad/sec].
        '''
        if (self.decoupling_enabled):
            self.decoupling_vq = speed * (bemf_const + Ld * id)
            self.decoupling_vd = -speed * Lq * iq

    def modified_speed(self, speed_max, torque_command_flag):
        '''
        Modifies the speed that's used in the MTPA LUT based on the voltage amplitude. \n
        If the voltage amplitude exceeds a threshold, the speed is artificially increased resulting in a lower voltage amplitude for the same or lower torque.
        Relevant only when commanding torque using a LUT.

        Args:

        '''
        if torque_command_flag:
            vs_err = self.vs - self.mod_speed_threshold

            self.mod_speed_integral_out += self.mod_speed_ki * vs_err * self.sampling_time
            self.mod_speed_integral_out = max(0, self.mod_speed_integral_out)                       # Prevent integral from becoming negative

            self.mod_speed_proportional_out = max(0, self.mod_speed_kp * vs_err)                    # Prevent the proportional output from becoming negative
            self.mod_speed_out = self.mod_speed_proportional_out + self.mod_speed_integral_out  
            
            if self.mod_speed_out > speed_max:
                self.mod_speed_out = speed_max                


class LUT:
    def __init__(self, config):
        '''
        Initializes lookup table generator related parameters:
        '''            
        self.torque_max = config.torque_max
        self.speed_max = config.speed_max        
        self.torque_increment = config.torque_increment
        self.speed_increment = config.speed_increment
        self.mtpa_lut = []


    def mtpa_cost_func(self, x, curr, we, vmax, motor):
        '''
        Finds max torque within current and voltage constraints \n
        Args:
            x - angle [rad]
            curr - Current amplitude [A]
            we - Electrical speed [rad/sec]
            vmax - Max allowed voltage amplitude [V]
            torque_const - Torque constant [Nm/A]
            Lq - Modified q axis inductance (Based on current) [uH]
            Ld - Modified d axis inductance (Based on current) [uH]
        Return:
            cost
        '''    
        ang = x[0]
        iq = curr * sin(ang)
        id = -curr * cos(ang)
        torque = abs(motor.pole_pairs * (3/2) * (motor.flux_linkage*iq + (motor.Ld - motor.Lq)*id*iq))
        if (torque != 0):
            cost = 1/torque
        else:
            cost = 1
        
        # Constraints:
        # Iq > 0
        if iq < 0:
            cost *= 2 ** (1 - iq)

        # Id < 0
        if id > 0:
            cost *= 2 ** (1 + id)
        
        # Voltage amplitude constraint: sqrt(Vq^2 + Vd^2) < VBus
        Vd = (motor.Rs*id - we*iq*motor.Lq)
        Vq = (motor.Rs*iq + we*(id*motor.Ld + motor.bemf_const / motor.pole_pairs))        
        VSize = np.sqrt(Vq**2 + Vd**2)
        if (VSize > vmax):
            cost *= 2 ** (1 + 10*(VSize/vmax))

        return abs(cost)



    def mtpa_gen(self, motor, app):
        '''
        Runs mtpa_cost_func() at all relevant points to get a LUT
        Args:

        motor class
        app class

        Return:
            MTPA LUT (rows = desired torque, columns = speed, cells = [iq, id, actual torque])
        '''      
        torque_list = np.arange(0,self.torque_max + self.torque_increment, self.torque_increment)
        speed_list = np.arange(0,self.speed_max + self.speed_increment, self.speed_increment)

        current_list = np.linspace(0,motor.i_max, int(len(torque_list) * motor.i_max / self.torque_max))
        vmax = app.v_batt_init / np.sqrt(3)    # Max phase voltage w/o overmodulation assuming third harmonic injection
        
        current_lut = [[[0, 0, 0] for _ in speed_list] for _ in current_list]
        self.mtpa_lut = [[[0, 0] for _ in speed_list] for _ in torque_list]

        # Finding accurate flux linkage according to motor datasheet.
        # If max torque is unknown, don't update the flux linkage.    
        motor.inductance_dq(motor.i_max, 0)
        for iter in range(10):
            res = minimize(self.mtpa_cost_func, np.pi/2 * 0.8, method='Nelder-Mead', tol=0.001, options={'maxiter': 200, 'disp': False}, args= (motor.i_max, 0, vmax, motor))
            res_torque_max = 1/res.fun
            motor.flux_linkage = motor.flux_linkage * (self.torque_max / res_torque_max)
        
        for curr_index in range(len(current_list)):   
            motor.inductance_dq(current_list[curr_index], 0)     
            x0 = np.linspace(np.pi / 50, np.pi/2, 5)
            for speed_index in range(len(speed_list)):
                elec_speed = speed_list[speed_index] * motor.pole_pairs
                if speed_index > 0:
                    id_min = (vmax/elec_speed - motor.bemf_const / motor.pole_pairs)/motor.Ld  # Minimal required Id current for PMSM
                    if id_min < 0:
                        if current_list[curr_index] < -id_min:
                            current_lut[curr_index][speed_index] = [0, id_min, 0]
                            continue

                if curr_index > 0:                
                    res = []
                    cost = []
                    for iter in range(5):                    
                        res.append(minimize(self.mtpa_cost_func, x0[iter], method='Nelder-Mead', tol=0.001, options={'maxiter': 100, 'disp': False}, args= (current_list[curr_index], elec_speed, vmax, motor)))
                        cost.append(res[iter].fun)
                    
                    abs_cost = [abs(ele) for ele in cost]
                    min_cost = min(abs_cost)
                    min_index = abs_cost.index(min_cost)

                    angle = res[min_index].x[0]
                    iq = current_list[curr_index] * sin(angle)    
                    id = -current_list[curr_index] * cos(angle)
                    current_lut[curr_index][speed_index] = [iq, id, 1/min_cost]
                    x0 = np.linspace(angle / 5, angle, 5)

        # Fill mtpa_lut with values from current_lut
        torque_threshold = self.torque_increment / 10
        # Initialize a list to track the last valid Iq and Id for each speed index
        last_valid_values = [[0, 0, 0] for _ in speed_list]        
        for mtpa_row_index, mtpa_torque in enumerate(torque_list):
            for speed_index in range(len(speed_list)):
                # Filter rows in current_lut that meet the torque threshold
                filtered_rows = [
                    (r, current_lut[r][speed_index])  # Row index and corresponding cell
                    for r in range(len(current_lut))
                    if abs(current_lut[r][speed_index][2] - mtpa_torque) <= torque_threshold
                ]

                if filtered_rows:
                    # Extract Iq and Id from current_lut
                    iq, id, trq = filtered_rows[0][1]
                    # Update last valid values for this speed
                    last_valid_values[speed_index] = [iq, id, trq]

                else:
                    # If no valid rows, use the last valid values for this speed
                    iq, id, trq = last_valid_values[speed_index]        
                
                # Fill the corresponding cell in mtpa_lut
                self.mtpa_lut[mtpa_row_index][speed_index] = [iq, id]

        # Create scatter plot for all speeds
        plt.figure(figsize=(10, 8))

        # Iterate through speeds and plot each speed's points
        for speed_index, speed in enumerate(speed_list):
            # Extract Id and Iq values for the current speed (column)
            iq_values = [row[speed_index][0] for row in self.mtpa_lut]
            id_values = [row[speed_index][1] for row in self.mtpa_lut]

            # Plot with a unique color for each speed
            plt.scatter(id_values, iq_values, label=f"Speed: {int(speed * 60 / (2*np.pi))} RPM")        
            # plt.title(speed)
            # Add torque values as text annotations
            for id_val, iq_val, torque_val in zip(id_values, iq_values, torque_list):
                plt.text(id_val, iq_val, f"{torque_val:.1f} Nm", fontsize=9, ha='right', va='bottom')

        # Label axes
        plt.xlabel("Id (A)")
        plt.ylabel("Iq (A)")
        plt.title("Iq vs Id for All Speeds")
        plt.axhline(0, color='black', linewidth=0.8, linestyle='--')
        plt.axvline(0, color='black', linewidth=0.8, linestyle='--')

        # Add legend and grid
        plt.legend()
        plt.grid(True)
        plt.show()       


    def bilinear_interpolation(self, torque, speed):
        """
        Perform bilinear interpolation to find Iq and Id at a given torque and speed.

        Args:
            torque_incr (float): Increment of torque between rows in the mtpa_lut.
            speed_incr (float): Increment of speed between columns in the mtpa_lut.
            torque (float): Target torque value.
            speed (float): Target speed value.

        Returns:
            iq_interp, id_interp (float, float): Interpolated Iq, Id values.
        """
        # Find the indices and weights for torque
        torque_index_low = int(torque // self.torque_increment)
        torque_index_high = torque_index_low + 1
        torque_weight_high = (torque - torque_index_low * self.torque_increment) / self.torque_increment
        torque_weight_low = 1 - torque_weight_high

        # Find the indices and weights for speed
        speed_index_low = int(speed // self.speed_increment)
        speed_index_high = speed_index_low + 1
        speed_weight_high = (speed - speed_index_low * self.speed_increment) / self.speed_increment
        speed_weight_low = 1 - speed_weight_high

        # Retrieve the 4 surrounding points from the mtpa_lut
        iq11, id11 = self.mtpa_lut[torque_index_low][speed_index_low]
        iq12, id12 = self.mtpa_lut[torque_index_low][speed_index_high]
        iq21, id21 = self.mtpa_lut[torque_index_high][speed_index_low]
        iq22, id22 = self.mtpa_lut[torque_index_high][speed_index_high]

        # Bilinear interpolation for Iq
        iq_interp = (
            iq11 * torque_weight_low * speed_weight_low +
            iq12 * torque_weight_low * speed_weight_high +
            iq21 * torque_weight_high * speed_weight_low +
            iq22 * torque_weight_high * speed_weight_high
        )

        # Bilinear interpolation for Id
        id_interp = (
            id11 * torque_weight_low * speed_weight_low +
            id12 * torque_weight_low * speed_weight_high +
            id21 * torque_weight_high * speed_weight_low +
            id22 * torque_weight_high * speed_weight_high
        )

        return iq_interp, id_interp        


class LowPassFilter:
    def __init__(self, dt, cutoff_frequency):
        """
        Initialize the low-pass filter.
        
        Args:
            cutoff_frequency (float): The cutoff frequency of the filter [Hz].
            sampling_rate (float): The sampling rate of the input signal [Hz].
        """
        # Calculate the filter coefficient (exponential smoothing factor)
        self.alpha = dt / (dt + 1 / (2 * np.pi * cutoff_frequency))
        self.prev_output = 0  # Initialize the previous output value

    def filter(self, input_value):
        """
        Apply the low-pass filter to the input value.
        
        Args:
            input_value: The current input signal value.

        Returns:
            The filtered value.
        """
        # Exponential smoothing formula
        output = self.alpha * input_value + (1 - self.alpha) * self.prev_output
        self.prev_output = output
        return output
    
class IIRFilter:
    def __init__(self, b, a, init_val):
        """
        Initialize the IIR filter with given coefficients.

        Args:
            b: Numerator coefficients of the transfer function
            a: Denominator coefficients of the transfer function
            init_val: Initial value
        """
        self.b = np.array(b) / a[0]  # Normalize by a[0]
        self.a = np.array(a) / a[0]
        self.x = np.zeros(len(b))  # Initialize input buffer
        self.y = np.zeros(len(a))  # Initialize output buffer

        # Set all past inputs to initial_output
        self.x = np.full(len(b), init_val)
        # Set all past outputs based DC gain
        steady_state_y = init_val * np.sum(b) / np.sum(a)
        self.y = np.full(len(a), steady_state_y)

    def step(self, input_sample):
        """
        Process a single sample through the filter.

        Parameters:
        - input_sample: The current input sample

        Returns:
        - output_sample: The current filtered output sample
        """
        # Shift input and output buffers
        self.x = np.roll(self.x, 1)
        self.y = np.roll(self.y, 1)
        self.x[0] = input_sample

        # Compute the output using direct form II transposed
        output_sample = np.sum(self.b * self.x) - np.sum(self.a[1:] * self.y[1:])
        
        self.y[0] = output_sample

        return output_sample    


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
        va - ia * motor.Ra - motor.bemf_a - motor.Laa_dot * ia - motor.Lab_dot * ib - motor.Lac_dot * ic,
        vb - ib * motor.Rb - motor.bemf_b - motor.Lab_dot * ia - motor.Lbb_dot * ib - motor.Lbc_dot * ic,
        vc - ic * motor.Rc - motor.bemf_c - motor.Lac_dot * ia - motor.Lbc_dot * ib - motor.Lcc_dot * ic,
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


def terminal_voltage_with_deadtime(ia, ib, ic, pwm_signals_top, pwm_signals_bottom, vbus):
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
            va_terminal = vbus  # Top transistor's voltage (bus voltage)
    else:
        va_terminal = pwm_signals_top[0] * vbus

    # Phase B
    if pwm_signals_top[1] == 0 and pwm_signals_bottom[1] == 0:
        if ib > 0:
            vb_terminal = 0  # Bottom transistor's voltage (ground)
        else:
            vb_terminal = vbus  # Top transistor's voltage (bus voltage)
    else:
        vb_terminal = pwm_signals_top[1] * vbus

    # Phase C
    if pwm_signals_top[2] == 0 and pwm_signals_bottom[2] == 0:
        if ic > 0:
            vc_terminal = 0  # Bottom transistor's voltage (ground)
        else:
            vc_terminal = vbus  # Top transistor's voltage (bus voltage)
    else:
        vc_terminal = pwm_signals_top[2] * vbus

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

    plt.figure(3)
    T, yout = ctrl.step_response(CL_d)
    plt.plot(T,yout)
    plt.title('D axis CL step response', fontsize = 20)

    plt.figure(4)
    T, yout = ctrl.step_response(CL_q)
    plt.plot(T,yout)
    plt.title('Q axis CL step response', fontsize = 20)    
    plt.show()    





def extract_iq_id_tables():
    """
    Main function to extract IQ and ID tables from a text file.

    This function opens a GUI for file selection, processes the selected file,
    and extracts two 2D arrays representing IQ and ID tables.

    Returns:
    tuple: A tuple containing two numpy arrays (iq_table, id_table).
           Returns (None, None) if no valid arrays are found.
    """    
    tables = []
    result_window = None

    def browse_file():
        """
        Opens a file dialog for the user to select a text file.

        This function is called when the "Browse File" button is clicked in the GUI.
        It opens a file dialog and calls process_file() if a file is selected.
        """
        nonlocal tables, result_window
        file_path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if file_path:
            tables = process_file(file_path)
            root.quit()

    def process_file(file_path):
        """
        Processes the selected file and extracts IQ and ID tables.

        This function reads the content of the file, removes C-style and python style comments,
        Finds all the standalone numbers converts them to arrays, and displays them in a formatted manner in a new window.

        Args:
        file_path (str): The path to the selected file.

        Side effects:
        - Creates a new window to display the extracted arrays.
        - Closes the main GUI window after processing.
        """        
        with open(file_path, 'r') as file:
            content = file.read()

        # Remove C-style multi-line comments
        content = re.sub(r'/\*[\s\S]*?\*/', '', content)
        
        # Remove C-style single-line comments and Python-style comments
        content = re.sub(r'//.*|#.*', '', content)

        # Find all standalone numbers in the file
        numbers = re.findall(r'\b-?\d+(?:\.\d+)?\b', content)
        
        # Convert to float (or int if possible)
        numbers = [int(x) if x.isdigit() else float(x) for x in numbers]
        
        # Determine the number of columns (assume it's the length of the first row)
        lines = content.split('\n')
        num_cols = 0
        for line in lines:
            nums_in_line = re.findall(r'\b-?\d+(?:\.\d+)?\b', line)
            if nums_in_line:
                num_cols = len(nums_in_line)
                break
        
        if num_cols == 0:
            return []  # No valid data found

        # Reshape the list into a 2D array
        num_rows = len(numbers) // num_cols
        table = np.array(numbers[:num_rows*num_cols]).reshape(num_rows, num_cols)

        # Split into two tables if there's an even number of rows
        if num_rows % 2 == 0:
            mid = num_rows // 2
            tables = [table[:mid], table[mid:]]
        else:
            tables = [table]

        display_results(tables)
        return tables

    def display_results(tables):
        """
        Display the extracted numeric tables in a new window.

        This function creates a new Tkinter window to show the contents of the
        extracted tables. Each table is displayed separately with its rows and
        columns preserved.

        Parameters:
        tables (list of numpy.ndarray): A list containing one or two 2D numpy arrays
                                        representing the extracted numeric tables.
        """        
        nonlocal result_window
        result_window = tk.Toplevel(root)
        result_window.title("Extracted Arrays")
        result_window.geometry("800x600")

        text_widget = scrolledtext.ScrolledText(result_window, wrap=tk.NONE)
        text_widget.pack(expand=True, fill='both')

        for i, table in enumerate(tables):
            text_widget.insert(tk.END, f"Array {i+1}:\n")
            for row in table:
                text_widget.insert(tk.END, " ".join(f"{val:8g}" for val in row) + "\n")
            text_widget.insert(tk.END, f"\nThis array has been assigned to 'table_{i+1}'\n\n")

        text_widget.config(state=tk.DISABLED)

    root = tk.Tk()
    root.title("Array Extractor")
    root.geometry("400x150")

    label = tk.Label(root, text="Please choose a text file with numeric data")
    label.pack(pady=10)

    browse_button = tk.Button(root, text="Browse File", command=browse_file)
    browse_button.pack(pady=10)

    root.mainloop()
    
    if result_window:
        root.wait_window(result_window)
    
    root.destroy()

    return tables if tables else [None, None]


# Lists for plotting:
speed_list = []
iqd_ramped_list = []
iqd_sensed_list = []
error_list = []
vqd_list = []
pi_outputs = []
pi_integral = []
pi_proportional = []
vabc_sine_mod_list = []
vabc_list = []
pwm_list = []
v_terminal = []
v_bus = []
i_bus_list = []
v_batt_list = []
i_batt_list = []
bemf = []
currents = []    
currents_filt = []
torque_commanded_list = []
torque_sensed_list = []
angle_list = []
dq_inductance_list = []
self_inductance_list = []
mutual_inductance_list = []
self_inductance_dot_list = []
mutual_inductance_dot_list = []
phase_volt_diff = []
phase_volt_diff_sine_mod = []
voltage_amplitude = []
afc_integrals = []
afc_outputs = []
decoupling_outputs = []
mod_speed_integral = []
mod_speed_proportional = []
mod_speed_output = []


def simulate_motor(motor, sim, app, control, lut, config):
    # Initializations
    speed_m = app.init_speed
    speed_e = speed_m * motor.pole_pairs
    angle_m = 0
    angle_e = 0
    iq_ramped = 0
    id_ramped = 0
    torque_ramped = 0
    ia, ib, ic = 0, 0, 0
    ia_filt, ib_filt, ic_filt = 0, 0, 0
    torque = 0
    battery_dv = 0

    # Assuming the phase currents are filtered before being read by the controller.
    ia_lpf = LowPassFilter(sim.time_step, config.phase_current_cutoff_freq)
    ib_lpf = LowPassFilter(sim.time_step, config.phase_current_cutoff_freq)
    ic_lpf = LowPassFilter(sim.time_step, config.phase_current_cutoff_freq)
    
    # Modeling the filter between the battery and the inverter:
    # Assuming an RLC network:
    # A resistor in series w/ an inductor modeling the cables connecting the two.
    # A resistor in series with a capacitor modeling the DC link capacitors
    # Vin --- R1 --- L ---+--- Vout
    #                     |
    #                     C
    #                     |
    #                     R2
    #                     |
    #                    GND
    # Transfer function:
    #   H(s) = (1 + s*C*R2) / (1 + s*C*(R2 + R1) + s^2*C*L)

    R1 = app.battery_resistance + config.cable_resistance
    L = config.battery_inductance + config.cable_inductance
    C = config.dc_link_capacitance
    R2 = config.dc_link_resistance

    # Define the continuous-time transfer function H(s)
    num = [C * R2, 1]
    den = [L * C, C * (R1 + R2), 1]
    H_s = ctrl.tf(num, den)

    # Discretize the system
    H_z = H_s.sample(sim.time_step, method='backward_diff')

    # Extract numerator and denominator coefficients from the discrete system
    num_z = H_z.num[0][0]
    den_z = H_z.den[0][0]

    # The filter affects the voltage and current passing between the battery and inverter
    dc_link_curr_filt = IIRFilter(num_z, den_z, 0)
    dc_link_volt_filt = IIRFilter(num_z, den_z, app.v_batt_init)

    for t in tqdm(sim.time_points, desc="Running simulation", unit=" Cycles"):
        # Ramp handling
        # Speed ramp
        if app.speed_control:
            if (speed_m < app.commanded_speed) and (app.acceleration != 0):            
                speed_m += app.acceleration * sim.time_step
            else:
                speed_m = app.commanded_speed

        else:   # app.speed_control == False
            speed_m += ((torque - speed_m * motor.visc_fric_coeff) / motor.inertia) * sim.time_step
        speed_e = speed_m * motor.pole_pairs
        speed_list.append([speed_m, speed_e])

        # Torque/Current ramps
        if (app.torque_command_flag):
            if (abs(torque_ramped) < abs(app.commanded_torque)) and (app.torque_ramp != 0):
                torque_ramped += app.torque_ramp * sim.time_step * np.sign(app.commanded_torque)
            else:
                torque_ramped = app.commanded_torque
            iq_ramped, id_ramped = lut.bilinear_interpolation(torque_ramped, min(lut.speed_max, speed_m + control.mod_speed_out))

        else:   # app.torque_command_flag == False
            if (abs(iq_ramped) < abs(app.commanded_iq)) and (app.current_ramp != 0):
                iq_ramped += app.current_ramp * sim.time_step * np.sign(app.commanded_iq)
            else:
                iq_ramped = app.commanded_iq

            if (abs(id_ramped) < abs(app.commanded_id)) and (app.current_ramp != 0):
                id_ramped += app.current_ramp * sim.time_step * np.sign(app.commanded_id)
            else:
                id_ramped = app.commanded_id        

        torque_commanded_list.append(torque_ramped)
        iqd_ramped_list.append([iq_ramped, id_ramped])

        # Convert abc frame currents to dq currents
        iq_sensed, id_sensed = dq_transform(ia_filt, ib_filt, ic_filt, angle_e)
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

            # Decoupling
            control.decoupling(iq_ramped, id_ramped, speed_e, motor.flux_linkage, motor.Ld, motor.Lq)

            control.vq = control.pi_vq + control.afc_vq + control.decoupling_vq
            control.vd = control.pi_vd + control.afc_vd + control.decoupling_vd
            control.vs = np.sqrt(control.vd ** 2 + control.vq ** 2)

            # Modified speed protection
            control.modified_speed(lut.speed_max, app.torque_command_flag)

            control.last_update_time = t      
                    
        vqd_list.append([control.vq, control.vd])
        pi_outputs.append([control.pi_vq, control.pi_vd])
        pi_integral.append([control.pi_integral_out_q, control.pi_integral_out_d])
        pi_proportional.append([control.pi_proportional_out_q, control.pi_proportional_out_d])
        afc_integrals.append([control.afc_sin_integral_error_d, control.afc_sin_integral_error_q, control.afc_cos_integral_error_d, control.afc_cos_integral_error_q])
        afc_outputs.append([control.afc_vq, control.afc_vd, control.afc_iq, control.afc_id])        
        decoupling_outputs.append([control.decoupling_vq, control.decoupling_vd])
        voltage_amplitude.append([control.vs])
        mod_speed_integral.append([control.mod_speed_integral_out])
        mod_speed_proportional.append([control.mod_speed_proportional_out])
        mod_speed_output.append([control.mod_speed_out])
        
        # Convert Vdq voltages to abc frame
        va_sineMod_unclipped, vb_sineMod_unclipped, vc_sineMod_unclipped = inverse_dq_transform(control.vq, control.vd, angle_e)

        va_sineMod = max(min(va_sineMod_unclipped, control.max_phase_v), -control.max_phase_v)
        vb_sineMod = max(min(vb_sineMod_unclipped, control.max_phase_v), -control.max_phase_v)
        vc_sineMod = max(min(vc_sineMod_unclipped, control.max_phase_v), -control.max_phase_v)
        vabc_sine_mod_list.append([va_sineMod, vb_sineMod, vc_sineMod])
        
        # Add third harmonic approx. to sinusoidal modulation.
        va_unclipped, vb_unclipped, vc_unclipped = third_harmonic(va_sineMod_unclipped, vb_sineMod_unclipped, vc_sineMod_unclipped, control.mod_fact)
        va = max(min(va_unclipped, control.max_phase_v), -control.max_phase_v)
        vb = max(min(vb_unclipped, control.max_phase_v), -control.max_phase_v)
        vc = max(min(vc_unclipped, control.max_phase_v), -control.max_phase_v)        
        vabc_list.append([va, vb, vc])

        phase_volt_diff.append([va-vb, vb-vc, vc-va])
        phase_volt_diff_sine_mod.append([va_sineMod-vb_sineMod, vb_sineMod-vc_sineMod, vc_sineMod-va_sineMod])

        # Calculate transistor values including dead time        
        # Short circuit the phases at half the sim time (Arbitrary) if short_circuit == True
        if (app.short_circuit == False): # or ((app.short_circuit == True) and (t < (sim.total_time / 2))):
            pwm_signals_top, pwm_signals_bottom = center_aligned_pwm_with_deadtime(va, vb, vc, control.max_phase_v, t, control.sampling_time, control.half_sampling_time, control.dead_time) 
        else:
            pwm_signals_top = [0, 0, 0]
            pwm_signals_bottom = [1, 1, 1]                    

        pwm_list.append([pwm_signals_top, pwm_signals_bottom])

        # Calculate terminal voltages including dead time (Terminal voltage are the voltages commanded by the drive unit, not the actual phase voltages.)
        va_terminal, vb_terminal, vc_terminal = terminal_voltage_with_deadtime(ia, ib, ic, pwm_signals_top, pwm_signals_bottom, app.v_bus)
        v_terminal.append([va_terminal, vb_terminal, vc_terminal])

        # Update Ld, Lq
        motor.inductance_dq(iq_sensed, -id_sensed)
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
        ia_filt = ia_lpf.filter(ia)
        ib_filt = ib_lpf.filter(ib)
        ic_filt = ic_lpf.filter(ic)

        currents.append([ia, ib, ic])
        currents_filt.append([ia_filt, ib_filt, ic_filt])

        torque_sensed = motor.torque(iq_sensed, id_sensed)
        torque_sensed_list.append(torque_sensed)

        angle_m += speed_m * sim.time_step
        angle_e += speed_e * sim.time_step
        angle_list.append([angle_m, angle_e])

        # Calculating power draw
        power_motor = ia * va_terminal + ib * vb_terminal + ic * vc_terminal
        power_conduction = config.transistor_resistance * (ia**2 + ib**2 + ic**2)
        power_switching = 6 * control.sampling_frequency * config.switch_energy_loss
        power_battery = power_motor + power_conduction + power_switching        

        i_bus = power_battery / app.v_bus
        i_bus_list.append(i_bus)
        i_batt = dc_link_curr_filt.step(i_bus)
        i_batt_list.append(i_batt)    

        # Updating the bus voltage based on a simplified model of a battery, a capacitor and an internal resistance
        battery_dv += (i_batt / app.battery_capacitance) * sim.time_step        
        v_batt = app.v_batt_init - (battery_dv + i_batt * app.battery_resistance)
        v_batt_list.append(v_batt)
        app.v_bus = dc_link_volt_filt.step(v_batt) - i_batt * config.cable_resistance
        v_bus.append(app.v_bus)

        # Updating some parameters which are a function of vbus:
        control.pi_v_lim = app.v_bus * 0.65
        control.max_phase_v = app.v_bus / 2
        control.mod_speed_threshold = app.v_bus / np.sqrt(3)



# Instantiate objects
config = Config()
motor = Motor(config)
sim = Simulation(config)
app = Application(config)
control = MotorControl(config)
lut = LUT(config)

# Uncomment to show closed loop bode plots of q and d axes:
# estimate_BW(control, app)

# If direct torque command is desired, a LUT is required. Generate or upload.
if (app.torque_command_flag):
    if (app.generate_lut):
        # Calculates this motor's MTPA LUT
        lut.mtpa_gen(motor, app)
    else:
        # Upload LUTs from text file
        iq_table, id_table = extract_iq_id_tables()
        lut.mtpa_lut = np.stack((iq_table, -id_table), axis=-1) / 10


# Run the simulation
simulate_motor(motor, sim, app, control, lut, config)


# Plot results
time_points = sim.time_points
speed_list = np.array(speed_list)
iqd_ramped_list = np.array(iqd_ramped_list)
iqd_sensed_list = np.array(iqd_sensed_list)
error_list = np.array(error_list)
vqd_list = np.array(vqd_list)
pi_outputs = np.array(pi_outputs)
pi_integral = np.array(pi_integral)
pi_proportional = np.array(pi_proportional)
vabc_sine_mod_list = np.array(vabc_sine_mod_list)
vabc_list = np.array(vabc_list)
pwm_list = np.array(pwm_list)
v_terminal = np.array(v_terminal)
v_bus = np.array(v_bus)
i_bus_list = np.array(i_bus_list)
v_batt_list = np.array(v_batt_list)
i_batt_list = np.array(i_batt_list)
bemf = np.array(bemf)
currents = np.array(currents)
currents_filt = np.array(currents_filt)
torque_commanded_list = np.array(torque_commanded_list)
torque_sensed_list = np.array(torque_sensed_list)
angle_list = np.array(angle_list)
dq_inductance_list = np.array(dq_inductance_list)
self_inductance_list = np.array(self_inductance_list)
mutual_inductance_list = np.array(mutual_inductance_list)
self_inductance_dot_list = np.array(self_inductance_dot_list)
mutual_inductance_dot_list = np.array(mutual_inductance_dot_list)
phase_volt_diff = np.array(phase_volt_diff)
phase_volt_diff_sine_mod = np.array(phase_volt_diff_sine_mod)
voltage_amplitude = np.array(voltage_amplitude)
voltage_limit = np.ones_like(time_points) * app.v_bus / np.sqrt(3)
afc_integrals = np.array(afc_integrals)
afc_outputs = np.array(afc_outputs)
decoupling_outputs = np.array(decoupling_outputs)
mod_speed_integral = np.array(mod_speed_integral)
mod_speed_proportional = np.array(mod_speed_proportional)
mod_speed_output = np.array(mod_speed_output)



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
    "v_bus": v_bus,
    "i_bus": i_bus_list,
    "v_batt": v_batt_list,
    "i_batt": i_batt_list,
    "bemf_a": bemf[:, 0],
    "bemf_b": bemf[:, 1],    
    "bemf_c": bemf[:, 2],        
    "ia": currents[:, 0],
    "ib": currents[:, 1],    
    "ic": currents[:, 2],
    "ia_filt": currents_filt[:, 0],
    "ib_filt": currents_filt[:, 1],    
    "ic_filt": currents_filt[:, 2],    
    "torque_commanded": torque_commanded_list,
    "torque_sensed": torque_sensed_list,
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
    "decoupling_vq": decoupling_outputs[:, 0],
    "decoupling_vd": decoupling_outputs[:, 1],
    "pi_vq": pi_outputs[:, 0],
    "pi_vd": pi_outputs[:, 1],
    "pi_integral_q": pi_integral[:, 0],
    "pi_integral_d": pi_integral[:, 1],
    "pi_proportional_q": pi_proportional[:, 0],
    "pi_proportional_d": pi_proportional[:, 1],
    "mod_speed_integral": mod_speed_integral,
    "mod_speed_proportional": mod_speed_proportional,
    "mod_speed_output": mod_speed_output,
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
Add support for induction motors.
'''
