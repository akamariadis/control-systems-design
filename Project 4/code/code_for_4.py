import numpy as np
import scipy.linalg as la
import scipy.signal as signal
import matplotlib.pyplot as plt

M = 1.0;
m = 0.1;
l = 0.5;
g = 9.81
den = 16 * M + 5 * m
A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, -27 * m * g / den, 0, 0],
    [0, 18 * (M + 2 * m) * g / (l * den), 0, 0]
])
B = np.array([[0], [0], [16 / den], [-18 / (l * den)]])
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Μετράμε x και θ
Q = np.diag([10, 1000, 1, 10])
R = np.array([[0.1]])
P = la.solve_continuous_are(A, B, Q, R)
K = la.inv(R) @ B.T @ P
cl_poles = la.eigvals(A - B @ K)
obs_poles = np.array([-15, -16, -17, -18])
res = signal.place_poles(A.T, C.T, obs_poles)
L = res.gain_matrix.T

print("Observer Gain L =\n", np.round(L, 2))

dt = 0.005
time = np.arange(0, 3, dt)
n_steps = len(time)
w = np.zeros((4, n_steps))
w_hat = np.zeros((4, n_steps))
w[:, 0] = [0.0, 0.1, 0.0, 0.0]
w_hat[:, 0] = [0.0, 0.0, 0.0, 0.0]

for i in range(1, n_steps):
    noise = np.random.normal(0, 0.005, 2)  # Τυπική απόκλιση 0.005
    y_noisy = C @ w[:, i - 1] + noise
    u = -K @ w_hat[:, i - 1]
    dw = A @ w[:, i - 1] + B @ u
    w[:, i] = w[:, i - 1] + dw * dt
    dw_hat = A @ w_hat[:, i - 1] + B @ u + L @ (y_noisy - C @ w_hat[:, i - 1])
    w_hat[:, i] = w_hat[:, i - 1] + dw_hat * dt

plt.figure(figsize=(10, 5))
plt.plot(time, w[3, :], label='Πραγματική Γωνιακή Ταχύτητα (w_4)', color='blue', linewidth=2)
plt.plot(time, w_hat[3, :], label='Εκτιμώμενη Γωνιακή Ταχύτητα (w_hat_4)', color='orange', linestyle='--', linewidth=2)
plt.title('Απόδοση Παρατηρητή Luenberger παρουσία θορύβου')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Γωνιακή Ταχύτητα (rad/s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()