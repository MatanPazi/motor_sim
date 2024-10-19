import numpy as np
import matplotlib.pyplot as plt

# Inverse Park Transformation
def inverse_park_transform(q, d, angle):
    
    a =  d * np.sin(angle) + q * np.cos(angle)
    b =  d * np.sin(angle - 2*np.pi/3) + q * np.cos(angle - 2*np.pi/3)
    c =  d * np.sin(angle + 2*np.pi/3) + q * np.cos(angle + 2*np.pi/3)
    return a, b, c

# Park Transformation
def park_transform(a, b, c, angle):
    d = (2/3) * (a * np.sin(angle) + b * np.sin(angle - 2*np.pi/3) + c * np.sin(angle + 2*np.pi/3))
    q = (2/3) * (a * np.cos(angle) + b * np.cos(angle - 2*np.pi/3) + c * np.cos(angle + 2*np.pi/3))    
    return q, d

# Test scenario
theta = np.linspace(0, 20, 1000)
ia_list = []
ib_list = []
ic_list = []
bemf_a_list = []
bemf_b_list = []
bemf_c_list = []
L_a_list = []
L_b_list = []
L_c_list = []
iq_transformed_list = []
id_transformed_list = []

# Set Iq = 10 and Id = 0
iq = 10
id = 0
bemf_const = 5
Ld = 10
Lq = 15

for ang in theta:
    # Apply inverse Park transformation to convert from dq to abc
    ia, ib, ic = inverse_park_transform(iq, id, ang)
    ia_list.append(ia)
    ib_list.append(ib)
    ic_list.append(ic)

    bemf_a = bemf_const * np.cos(ang)
    bemf_b = bemf_const * np.cos(ang - 2*np.pi/3)
    bemf_c = bemf_const * np.cos(ang + 2*np.pi/3)
    bemf_a_list.append(bemf_a)
    bemf_b_list.append(bemf_b)
    bemf_c_list.append(bemf_c)

    L_a = Lq * (np.cos(ang))**2 + Ld * (np.sin(ang))**2
    L_b = Lq * (np.cos(ang - 2*np.pi/3))**2 + Ld * (np.sin(ang - 2*np.pi/3))**2
    L_c = Lq * (np.cos(ang + 2*np.pi/3))**2 + Ld * (np.sin(ang + 2*np.pi/3))**2
    L_a_list.append(L_a)
    L_b_list.append(L_b)
    L_c_list.append(L_c)    

    # Apply Park transformation to convert back from abc to dq
    iq_transformed, id_transformed = park_transform(ia, ib, ic, ang)
    iq_transformed_list.append(iq_transformed)
    id_transformed_list.append(id_transformed)

# Plot the results
plt.figure(1)
plt.plot(theta, iq_transformed_list, label='Iq Transformed (Expected 10)')
plt.plot(theta, id_transformed_list, label='Id Transformed (Expected 0)')
plt.legend()


plt.figure(2)
plt.plot(theta, ia_list, label='Ia')
plt.plot(theta, ib_list, label='Ib')
plt.plot(theta, ic_list, label='Ib')
plt.plot(theta, bemf_a_list, label='bemf_a')
plt.plot(theta, bemf_b_list, label='bemf_b')
plt.plot(theta, bemf_c_list, label='bemf_c')
plt.plot(theta, L_a_list, label='L_a')
plt.plot(theta, L_b_list, label='L_b')
plt.plot(theta, L_c_list, label='L_c')
plt.legend()
plt.show()
