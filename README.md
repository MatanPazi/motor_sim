# Motor Simulator

This repository contains a Python-based motor simulator designed to aid in control algorithm development. The simulation allows fine-grained control over motor and application settings and supports real-time switching behavior and dead-time effects.

## Features

- Simulates motor electrical and mechanical dynamics for Synchronous motors (Asynchronous TBD)
- Takes into account mutual and self inductances, as well as inductance saturation and inductance time derivatives
- Accounts for dead-time and switching behavior
- Supports back-emf harmonics
- Allows the simulation of short-circuiting all phases

## How It Works

The simulator integrates electrical and mechanical motor equations to simulate motor behavior.
A current vector is commanded, and following a current loop, the FOC voltages Vd and Vq are calculated.
These voltages are transformed to transistor states (Taking dead-time into account).
The phase currents are found by solving the voltage equations.
And the user can plot anything they wish:
- Phase currents
- Phase voltages
- Torque
- Speed
- Inductances (Self, mutual, dq)
- etc.

## Usage

Modify parameters by directly adjusting the class initialization in the script:

```python
class Motor:
    def __init__(self, motor_type="SYNC", pole_pairs=4, Rs=0.0029, Lq_base=0.0000685, Ld_base=0.0000435,
                 bemf_const=0.1, inertia=0.0091, visc_fric_coeff=0.005, i_max = 600):

class MotorControl:
    def __init__(self, Kp_d=0.2, Ki_d=50.0, Kp_q=0.2, Ki_q=50.0, sampling_time=62.5e-6, dead_time = 300e-9):

class Application:
    def __init__(self, speed_control=True, commanded_speed=10.0, commanded_iq=10.0, commanded_id=0.0,
                 acceleration=10000.0, current_ramp=10000.0, vBus = 48, init_speed = 0, short_circuit = False):

class Simulation:
    def __init__(self, time_step=100e-9, total_time=0.002):
```
Parameter units and explanations are in the script.

## Running the simulation, example:
```
# Instantiate objects
motor = Motor()
sim = Simulation()
app = Application()
control = MotorControl()

# Uncomment to show closed loop bode plots of q and d axes:
# estimate_BW()

# Run the simulation
simulate_motor(motor, sim, app, control)

# Plot results
...
```
![WhatsApp Image 2024-10-15 at 09 22 12](https://github.com/user-attachments/assets/7b036f18-9923-4fde-bd1e-d25c9c807a72)


![WhatsApp Image 2024-10-15 at 09 22 12 (1)](https://github.com/user-attachments/assets/bb94a8ae-bd7f-450d-aa14-84b87683151c)

![Figure_1](https://github.com/user-attachments/assets/569e7582-b96a-4df7-8a50-f53a09151bec)


## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Miscellaneous
- Inspired by Ben Katz's motor simulator:
  https://build-its-inprogress.blogspot.com/2016/12/some-motor-math-and-simulation.html.
  I saw he built a motor simulator in MATLAB which made me want to build a motor simulator in Python.

- Many thanks to ChatGPT for helping me write this README file :).
