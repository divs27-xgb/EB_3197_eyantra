"""Provisional Task 2A bike LQR model from the corrected contact constraint.

This calculation is a transparent candidate, not a validated tuner model.
Run with Python + NumPy. It does not connect to CoppeliaSim or send commands.
"""
import numpy as np

# Scene-derived estimates (SI units)
M_cluster = np.array([
    [0.0036549, -0.0026676],
    [-0.0026676, 0.0146332],
])  # includes rigid-body lean/yaw inertia of the front wheel bodies
J_front_spin = 0.00054           # two dynamic front spherical components, kg m^2
L = 0.200                         # pivot to front contact centre, m
r = 0.015                         # wheel radius, m
m_body = 0.600                    # kg
h_body = 0.03176                   # body COM height above spherical pivot, m
Kv = 0.100                         # provisional velocity-servo P gain, N m/(rad/s)

k = L / r
# phi_dot = -theta_dot - k*psi_dot.
# Replace front bodies' baseline J*theta_dot^2 term with J*k^2*psi_dot^2.
M = M_cluster + np.diag([-J_front_spin, J_front_spin * k**2])
G = np.diag([-m_body * 9.81 * h_body, 0.0])
D = Kv * np.array([[1.0, k], [k, k**2]])
b = Kv * np.array([-1.0, -k])

# q = [theta, psi], x = [theta_dot, theta, psi_dot, psi]
accel_from_qdot = np.linalg.solve(M, -D)
accel_from_q = np.linalg.solve(M, -G)
accel_from_u = np.linalg.solve(M, b)
A = np.array([
    [accel_from_qdot[0, 0], accel_from_q[0, 0], accel_from_qdot[0, 1], accel_from_q[0, 1]],
    [1.0, 0.0, 0.0, 0.0],
    [accel_from_qdot[1, 0], accel_from_q[1, 0], accel_from_qdot[1, 1], accel_from_q[1, 1]],
    [0.0, 0.0, 1.0, 0.0],
])
B = np.array([[accel_from_u[0]], [0.0], [accel_from_u[1]], [0.0]])

np.set_printoptions(precision=6, suppress=True)
print(f"k = {k:.6f}")
print("M =\n", M)
print("G =\n", G)
print("D =\n", D)
print("b =", b)
print("A =\n", A)
print("B =\n", B)
print("A eigenvalues =", np.linalg.eigvals(A))
print("NOTE: signs assume theta about +Y, psi about +Z, positive phi about +Y, ideal lateral no-slip, and tau=Kv*(u-phi_dot).")
