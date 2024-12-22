import control as ctrl
import matplotlib.pyplot as plt
import numpy as np

R = 1

# Motor transfer function: G(s) = I/V = 1/R ->
num_R = [1]
den_R = [R]

# Create transfer functions
G_R = ctrl.TransferFunction(num_R, den_R)

_, yout = ctrl.step_response(G_R)
yout = np.insert(yout,0,0)
plt.plot(yout, label = 'Current response')
plt.title('Current response to a voltage step command')
plt.legend()
plt.show()















L = 0.001
# # Motor transfer function: G(s) = I/V = 1 / (L * s + R)
num_RL = [1]
den_RL = [L, R]

# Create transfer functions
G_RL = ctrl.TransferFunction(num_RL, den_RL)

# _, yout = ctrl.step_response(G_RL)
# plt.plot(yout, label = 'Current response')
plt.title('Current response to a voltage step command')
# plt.legend()
# plt.show()