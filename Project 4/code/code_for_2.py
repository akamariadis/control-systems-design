import numpy as np
import scipy.linalg as la

M = 1.0
m = 0.1
l = 0.5
g = 9.81
den = 16 * M + 5 * m
A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, -27 * m * g / den, 0, 0],
    [0, 18 * (M + 2 * m) * g / (l * den), 0, 0]
])
B = np.array([
    [0],
    [0],
    [16 / den],
    [-18 / (l * den)]
])
Q = np.diag([10, 1000, 1, 10])
R = np.array([[0.1]])
P = la.solve_continuous_are(A, B, Q, R)
K = la.inv(R) @ B.T @ P
print("Ο πίνακας κέρδους LQR (K) είναι:")
print(np.round(K, 4))