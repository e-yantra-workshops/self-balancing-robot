import numpy as np
import control as ctrl
from control.matlab import ctrb, lqr

# System matrices
A = np.array([
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
])

B = np.array([
    [0],
    [0],
    [0],
    [0]
])

# LQR weighting matrices (correct dimensions)
Q = np.diag([1, 1, 1, 1])   # 4x4
R = np.array([[1]])                # 1x1

# Output matrices
C = np.eye(4)
D = np.zeros((4, 1))

# Create continuous-time state-space system
system = ctrl.StateSpace(A, B, C, D)

# Convert to transfer function
tf_conv = ctrl.ss2tf(system)

# Controllability check
controllability = ctrb(A, B)
print("Rank of Controllability Matrix:", np.linalg.matrix_rank(controllability))
print("Determinant of A:", np.linalg.det(A))

# LQR controller
K, S, E = lqr(A, B, Q, R)

print("LQR Gain K:")
print(np.array(K, dtype=float))
