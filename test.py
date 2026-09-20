import sympy as sp
import numpy as np
import control
theta, omega, u =sp.symbols('theta omega u')

# Define the differential equations

theta_dot=omega
omega_dot= -10*sp.sin(theta)-omega+u

# The order of the states. Keep it as angle first, then its rate.
STATES = [theta, omega]



def find_equilibrium_points():
    eq1=theta_dot.subs(u,0)
    eq2=omega_dot.subs(u,0)
    equi_points=sp.solve([eq1,eq2],[theta,omega])
    return equi_points

def find_A_B_matrices(eq_points):

    f = sp.Matrix([theta_dot, omega_dot])
    A_matrices, B_matrices = [], []
    A_sym=f.jacobian(STATES)
    B_sym=f.jacobian([u])
    for i in eq_points:
        A_matrices.append(A_sym.subs({theta: i[0], omega: i[1], u: 0}))
        B_matrices.append(B_sym.subs({theta: i[0], omega: i[1], u: 0}))
    return A_matrices, B_matrices


def find_eigen_values(A_matrices):
    eigen_values = []
    stability = []
    for i in A_matrices:
        eigen=sp.Matrix(i).eigenvals()
        eigen_values.append(eigen)
        for k in list(eigen):
            if sp.re(k)>=0:
                stability.append("Unstable")
                break
        else:
            stability.append("Stable")
    return eigen_values, stability
def compute_lqr_gain(A_matrices, B_matrices, stability):
    '''
    Design the controller for the one equilibrium that needs it: hanging
    down settles back on its own, balanced upright does not.

    Input Arguments:
    ---
    `A_matrices`, `B_matrices`: [ lists of sympy Matrix ]
    `stability`: [ list of str ] from find_eigen_values()

    Returns:
    ---
    `K`: [ numpy array ] the LQR gain, one entry per state

    Steps: pick out the A and B at the unstable equilibrium, convert both to
    float numpy arrays (control.lqr() will not take sympy Matrix objects),
    then call control.lqr(A, B, Q, R) and keep the first of its three
    return values.

    Note: do not change Q or R; they are fixed for Task 1A so every team's
    K is comparable.
    '''
    # Define the Q and R matrices
    K=[]
    Q = np.eye(2)        # State weighting matrix
    R = np.array([1])    # Control weighting matrix
    indices = [index for index, value in enumerate(stability) if value == "Unstable"]
    a_num=np.array(A_matrices).astype(float)
    b_num=np.array(B_matrices).astype(float)
    for i in indices:
        gain,_,_=control.lqr(a_num[i],b_num[i],Q,R)
        K.append(gain)


    # for i in indices:


    ###### WRITE YOUR CODE HERE ################
    # HINT: control.lqr() takes plain numpy arrays and returns three
    # things, the gain first, e.g.
    #     K, _, _ = control.lqr(A, B, Q, R)

    ############################################

    return K

eq_points=find_equilibrium_points()
a,b=find_A_B_matrices(eq_points)
# sp.pprint(a)
eig,stability=find_eigen_values(a)
# print(eig,stability,sep='\n')
# print(stability)
k=compute_lqr_gain(a,b,stability)
print(k)
