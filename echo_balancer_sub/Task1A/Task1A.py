'''
*****************************************************************************************
*
*        =================================================
*             Echo Balancer (EB) Theme (eYRC 2026-27)
*        =================================================
*
*  This script is to implement Task 1A of Echo Balancer (EB) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [eYRC#3197]
# Author List:      [ Priyansh Singh Rathore, Devesh Dev ]
# Filename:         Task1A.py
# Functions:        find_equilibrium_points, find_A_B_matrices,
#                   find_eigen_values, compute_lqr_gain
# Global variables: theta, omega, u, theta_dot, omega_dot, STATES

import sympy as sp
import numpy as np
import control

############################################################################
#                        THE SYSTEM  -  fill this in                       #
############################################################################
# The pendulum-with-torque system from the "Modeling of non-linear Dynamical
# Systems" Learnings page, with one added damping term:
#
#     theta_dot = omega
#     omega_dot = -10*sin(theta) - omega + u
#
#   theta = angle from vertical   omega = angular rate   u = applied torque
#
# Use sympy symbols, not plain Python numbers, so these expressions stay
# differentiable.

# Define the symbolic variables
theta, omega, u = sp.symbols('theta omega u')

# Define the differential equations

theta_dot = omega
omega_dot = -10 * sp.sin(theta) - omega + u

# The order of the states. Keep it as angle first, then its rate.
STATES = [theta, omega]


def find_equilibrium_points():
    eq1 = theta_dot.subs(u, 0)
    eq2 = omega_dot.subs(u, 0)
    equi_points = sp.solve([eq1, eq2], [theta, omega])
    return equi_points


def find_A_B_matrices(eq_points):
    f = sp.Matrix([theta_dot, omega_dot])
    A_matrices, B_matrices = [], []
    A_sym = f.jacobian(STATES)
    B_sym = f.jacobian([u])
    for i in eq_points:
        A_matrices.append(A_sym.subs({theta: i[0], omega: i[1], u: 0}))
        B_matrices.append(B_sym.subs({theta: i[0], omega: i[1], u: 0}))
    return A_matrices, B_matrices


def find_eigen_values(A_matrices):
    eigen_values = []
    stability = []

    for i in A_matrices:
        eigen = sp.Matrix(i).eigenvals()
        eigen_values.append(eigen)
        for k in list(eigen):
            if sp.re(k) >= 0:
                stability.append("Unstable")
                break
        else:
            stability.append("Stable")

    return eigen_values, stability


def compute_lqr_gain(A_matrices, B_matrices, stability):
    # Define the Q and R matrices
    K = []
    Q = np.eye(2)  # State weighting matrix
    R = np.array([1])  # Control weighting matrix
    indices = [index for index, value in enumerate(stability) if value == "Unstable"]
    a_num = np.array(A_matrices).astype(float)
    b_num = np.array(B_matrices).astype(float)
    for i in indices:
        gain, _, _ = control.lqr(a_num[i], b_num[i], Q, R)
        K.append(gain)

    return K


def main_function():  # Don't change anything in this function
    eq_points = find_equilibrium_points()

    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None, None

    A_matrices, B_matrices = find_A_B_matrices(eq_points)
    eigen_values, stability = find_eigen_values(A_matrices)
    K = compute_lqr_gain(A_matrices, B_matrices, stability)

    return eq_points, A_matrices, B_matrices, eigen_values, stability, K


def task1a_output(eq_points, A_matrices, B_matrices, eigen_values, stability, K):
    '''
    This function prints the results you have obtained.
    '''
    print("Equilibrium Points:")
    for i, point in enumerate(eq_points):
        print(f"  Point {i + 1}: theta = {point[0]}, omega = {point[1]}")

    print("\nA Matrices at Equilibrium Points:")
    for i, matrix in enumerate(A_matrices):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=False))

    print("\nB Matrices at Equilibrium Points:")
    for i, matrix in enumerate(B_matrices):
        print(f"  At Point {i + 1}: {sp.Matrix(matrix).T.tolist()[0]}")

    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        eigvals_str = ', '.join([f"{val}: {count}" for val, count in eigvals.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")

    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")

    print("\nLQR Gain Matrix K at the unstable Equilibrium Point:")
    print(K)


if __name__ == "__main__":
    results = main_function()
    eq_points, A_matrices, B_matrices, eigen_values, stability, K = results
    task1a_output(eq_points, A_matrices, B_matrices, eigen_values, stability, K)
