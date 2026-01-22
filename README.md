# Motor Simulator

This repository contains a Python-based motor simulator designed to aid in control algorithm development. The simulation allows fine-grained control over motor and application settings and supports real-time switching behavior and dead-time effects.

Here's a video I made about this script:  

**Motor Control Simulation: Theory & Implementation**  
[![Video Title](https://img.youtube.com/vi/CC1rBmhWIqo/0.jpg)](https://www.youtube.com/watch?v=CC1rBmhWIqo)

## Features

- Simulates motor electrical and mechanical dynamics for Synchronous motors (Asynchronous TBD)
- Takes into account mutual and self inductances, as well as inductance saturation and inductance time derivatives
- Accounts for dead-time and switching behavior
- Supports back-emf harmonics
- Allows the simulation of short-circuiting all phases
- Supports MTPA (Maximum torque per ampere) LUT (Lookup table) generation based on motor parameters
- Supports loading external MTPA LUTs
- Supports AFC (Adaptive feed forward cancellation) control for harmonic attenuation
- Supports decoupling
- Features a simple battery model
- Supports an input filter (RLC network) from the battery
- Supports unbalanced phases

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

Modify parameters by directly adjusting the config class initialization in the script:

```python
class Config:
    def __init__(self):
        '''
        Initializes all script parameters:

        Args:
        ...
```
Parameter units and explanations are in the script.

## Running the simulation, example:
```
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
simulate_motor(motor, sim, app, control, lut)

# Plot results
...
```
![WhatsApp Image 2024-10-15 at 09 22 12](https://github.com/user-attachments/assets/7b036f18-9923-4fde-bd1e-d25c9c807a72)


![WhatsApp Image 2024-10-15 at 09 22 12 (1)](https://github.com/user-attachments/assets/bb94a8ae-bd7f-450d-aa14-84b87683151c)

![Figure_1](https://github.com/user-attachments/assets/569e7582-b96a-4df7-8a50-f53a09151bec)


## Dependencies
This project requires the following Python modules:
- control: https://python-control.readthedocs.io/en/0.10.1/index.html
- scipy
- matplotlib
- numpy
- tqdm: https://tqdm.github.io/
- re
- tkinter
- math

### Note:
In the root folder, run `pip install -r requirements.txt` to easily install third-party python modules

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Miscellaneous
- Inspired by Ben Katz's motor simulator:
  https://build-its-inprogress.blogspot.com/2016/12/some-motor-math-and-simulation.html.
  I saw he built a motor simulator in MATLAB which made me want to build a motor simulator in Python.
