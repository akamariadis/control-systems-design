import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

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
x_ref = 1.0
w_ref = np.array([x_ref, 0.0, 0.0, 0.0])
def closed_loop_dynamics(t, w):
    u = -K @ (w - w_ref)
    dw = A @ w + B @ u
    return dw
w0 = np.array([0.0, 0.1, 0.0, 0.0])
t_span = (0, 5)
t_eval = np.linspace(t_span[0], t_span[1], 1000)
sol = solve_ivp(closed_loop_dynamics, t_span, w0, t_eval=t_eval)
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(sol.t, sol.y[0], label='Θέση Cart $x(t)$', color='blue', linewidth=2)
plt.axhline(x_ref, color='red', linestyle='--', label=f'Στόχος $x_{{ref}} = {x_ref}$ m')
plt.title('Χρονική Απόκριση Θέσης Cart')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Θέση (m)')
plt.grid(True, linestyle=':')
plt.legend()
plt.subplot(1, 2, 2)
theta_deg = np.degrees(sol.y[1])
plt.plot(sol.t, theta_deg, label='Γωνία Εκκρεμούς $\\theta(t)$', color='darkorange', linewidth=2)
plt.axhline(0, color='red', linestyle='--', label='Κατακόρυφος (0$^\circ$)')
plt.title('Χρονική Απόκριση Γωνίας Εκκρεμούς')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Γωνία (μοίρες)')
plt.grid(True, linestyle=':')
plt.legend()
plt.tight_layout()
plt.show()