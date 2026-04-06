import control as ctrl
import matplotlib.pyplot as plt
import numpy as np

# FIG 1
G = ctrl.TransferFunction([1], [1, 3, 2]) # s^2 + 3s + 2
ctrl.root_locus(G)
plt.show()

# FIG 2
num = [1]
den = [1, 3, 2]
sys = ctrl.TransferFunction(num, den)
plt.figure(figsize=(10, 8))
ctrl.root_locus(sys, plot=True)

# FIG 2
x_max = -4.0
x_min = -12.0
x_fill = np.linspace(x_min, x_max, 200)
zeta = 0.4
slope = np.sqrt(1 - zeta**2) / zeta
y_upper = -slope * x_fill
y_lower = slope * x_fill
plt.fill_between(x_fill, y_lower, y_upper, color='green', alpha=0.3, label='Επιθυμητή Περιοχή ($\zeta \geq 0.4$, $t_s < 1s$)')
plt.axvline(x=-4, color='red', linestyle='--', alpha=0.7, label='Όριο $t_s < 1s$ ($\sigma = -4$)')
x_line = np.linspace(-12, 0, 100)
plt.plot(x_line, -slope * x_line, 'b--', alpha=0.5, label='Όριο $\zeta = 0.4$')
plt.plot(x_line, slope * x_line, 'b--', alpha=0.5)
plt.xlim([-10, 1])
plt.ylim([-8, 8])
plt.title('Γεωμετρικός Τόπος Ριζών και Επιθυμητή Περιοχή Λειτουργίας')
plt.xlabel('Πραγματικός Άξονας (Real)')
plt.ylabel('Φανταστικός Άξονας (Imaginary)')
plt.legend(loc='lower right')
plt.grid(True)
plt.show()

# FIG 3
G = ctrl.TransferFunction([1], [1, 3, 2])
zc = 5.0
pc = 18.5
Kc = 107.5
C = Kc * ctrl.TransferFunction([1, zc], [1, pc])
T_actual = ctrl.feedback(C * G)
zeta = 5 / np.sqrt(5**2 + 5**2)
wn = np.sqrt(5**2 + 5**2)
T_ideal = ctrl.TransferFunction([wn**2], [1, 2*zeta*wn, wn**2])
t = np.linspace(0, 1.5, 500)
t_act, y_act = ctrl.step_response(T_actual, t)
t_id, y_id = ctrl.step_response(T_ideal, t)
overshoot_act = (np.max(y_act) - 1.0) * 100
overshoot_id = (np.max(y_id) - 1.0) * 100
plt.figure(figsize=(10, 6))
plt.plot(t_act, y_act, 'b-', linewidth=2, label=f'Πραγματικό Σύστημα με Ελεγκτή (Υπερύψωση: {overshoot_act:.1f}%)')
plt.plot(t_id, y_id, 'r--', linewidth=2, label=f'Ιδανικό Σύστημα 2ης Τάξης (Υπερύψωση: {overshoot_id:.1f}%)')
plt.axhline(y=1, color='k', linestyle=':', alpha=0.6, label='Επιθυμητή Τιμή (Βήμα)')
plt.title('Σύγκριση Βηματικής Απόκρισης: Αναμενόμενη vs Πραγματική Υπερύψωση', fontsize=14)
plt.xlabel('Χρόνος (s)', fontsize=12)
plt.ylabel('Πλάτος', fontsize=12)
plt.legend(loc='lower right')
plt.grid(True)
plt.show()

# FIG 4
num_G = [1]
den_G = [1, 3, 2]
sys_plant = ctrl.TransferFunction(num_G, den_G)
sys_controller = Kc * ctrl.TransferFunction([1, zc], [1, pc])
sys_open_loop = sys_controller * sys_plant
sys_cl_actual = ctrl.feedback(sys_open_loop, 1)
plt.figure(figsize=(12, 8))
mag_G, phase_G, omega = ctrl.bode(sys_plant, plot=False)
mag_C, phase_C, omega = ctrl.bode(sys_controller, omega, plot=False)
mag_L, phase_L, omega = ctrl.bode(sys_open_loop, omega, plot=False)
plt.subplot(2, 1, 1)
plt.semilogx(omega, 20*np.log10(mag_G), label='Φυτό $G(s)$', linestyle=':')
plt.semilogx(omega, 20*np.log10(mag_C), label='Ελεγκτής $C(s)$', linestyle='--')
plt.semilogx(omega, 20*np.log10(mag_L), label='Βρόχος $L(s) = C(s)G(s)$', linewidth=2)
plt.axhline(0, color='black', alpha=0.3) # Γραμμή 0 dB
plt.ylabel('Μέτρο (dB)')
plt.title('Διαγράμματα Bode: $G(s)$, $C(s)$ και $L(s)$')
plt.legend()
plt.grid(True, which="both", ls="-")
plt.subplot(2, 1, 2)
plt.semilogx(omega, np.degrees(phase_G), label='Φυτό $G(s)$', linestyle=':')
plt.semilogx(omega, np.degrees(phase_C), label='Ελεγκτής $C(s)$', linestyle='--')
plt.semilogx(omega, np.degrees(phase_L), label='Βρόχος $L(s) = C(s)G(s)$', linewidth=2)
plt.axhline(-180, color='red', alpha=0.5, label='Όριο Σταθερότητας -180°')
plt.ylabel('Φάση (μοίρες)')
plt.xlabel('Συχνότητα (rad/s)')
plt.legend()
plt.grid(True, which="both", ls="-")
plt.tight_layout()
gm, pm, wg, wp = ctrl.margin(sys_open_loop)
plt.show()

# Closed Loop Transfer Function
G = ctrl.TransferFunction([1], [1, 3, 2])
C = 51.6 * ctrl.TransferFunction([1, 3], [1, 10.5])
T = ctrl.feedback(C * G, 1)
F = ctrl.TransferFunction([3], [1, 3])
T_filtered = ctrl.minreal(F * T)
print("Κλειστός Βρόχος (Χωρίς Φίλτρο):\n", T)
print("Κλειστός Βρόχος (Με Φίλτρο):\n", T_filtered)
t = np.linspace(0, 2, 500)
t1, y1 = ctrl.step_response(T, t)
t2, y2 = ctrl.step_response(T_filtered, t)

# FIG 5
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(t1, y1, label='Χωρίς Φίλτρο', linewidth=2)
plt.plot(t2, y2, label='Με Φίλτρο (Ακύρωση Μηδενικού)', linewidth=2, linestyle='--')
plt.axhline(1, color='k', linestyle=':', alpha=0.5)
plt.title('Βηματική Απόκριση')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Πλάτος')
plt.legend()
plt.grid(True)
plt.subplot(1, 2, 2)
mag1, phase1, omega = ctrl.bode(T, plot=False)
mag2, phase2, omega = ctrl.bode(T_filtered, omega, plot=False)
plt.semilogx(omega, 20*np.log10(mag1), label='Χωρίς Φίλτρο', linewidth=2)
plt.semilogx(omega, 20*np.log10(mag2), label='Με Φίλτρο', linewidth=2, linestyle='--')
plt.axhline(0, color='k', linestyle=':', alpha=0.5)
plt.axhline(-3, color='r', linestyle=':', alpha=0.5, label='-3 dB (Bandwidth)')
plt.title('Διάγραμμα Bode Κλειστού Βρόχου (Μέτρο)')
plt.xlabel('Συχνότητα (rad/s)')
plt.ylabel('Μέτρο (dB)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# FIG 6
G = ctrl.TransferFunction([1], [1, 3, 2])
C = 51.62 * ctrl.TransferFunction([1, 3], [1, 10.517])
F = ctrl.TransferFunction([3], [1, 3])
Tu_no_filt = ctrl.feedback(C, G)
Tu_filt = ctrl.minreal(F * Tu_no_filt)
t = np.linspace(0, 1.5, 1000)
t1, u1 = ctrl.step_response(Tu_no_filt, t)
t2, u2 = ctrl.step_response(Tu_filt, t)
plt.figure(figsize=(10, 6))
plt.plot(t1, u1, 'b-', linewidth=2, label='Σήμα Ελέγχου $u(t)$ - Χωρίς Φίλτρο')
plt.plot(t2, u2, 'r--', linewidth=2.5, label='Σήμα Ελέγχου $u(t)$ - Με Φίλτρο')
plt.title('Σύγκριση Σήματος Ελέγχου $u(t)$', fontsize=14)
plt.xlabel('Χρόνος (s)', fontsize=12)
plt.ylabel('Πλάτος Σήματος Ελέγχου', fontsize=12)
plt.legend(loc='upper right')
plt.grid(True)
plt.ylim([-5, 60])
plt.tight_layout()
plt.show()

# FIG 7 - FIG 8
G1 = ctrl.TransferFunction([100], [1, 3, 2])
G2 = ctrl.TransferFunction([900], [1, 2, 900])
G = G1 * G2
plt.figure(figsize=(10, 6))
ctrl.bode(G, dB=True, margins=True)
plt.suptitle('Bode Diagram')
plt.figure(figsize=(8, 8))
ctrl.nyquist_plot(G)
plt.plot(-1, 0, 'r+', markersize=12, markeredgewidth=2, label='Κρίσιμο Σημείο (-1, j0)')
plt.xlim([-2, 1])
plt.ylim([-1.5, 1.5])
plt.legend()
plt.grid(True)
plt.show()
gm, pm, wg, wp = ctrl.margin(G)
if gm > 0 and gm != float('inf'):
    print(f"Περιθώριο Κέρδους (GM): {20 * np.log10(gm):.2f} dB")
else:
    print(f"Περιθώριο Κέρδους (GM): Μη διαθέσιμο / Άπειρο")
print(f"Περιθώριο Φάσης (PM): {pm:.2f} μοίρες")
print(f"Συχνότητα Phase Crossover (wg): {wg:.2f} rad/s")
print(f"Συχνότητα Gain Crossover (wp): {wp:.2f} rad/s")