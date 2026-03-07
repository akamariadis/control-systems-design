% ΜΕΡΟΣ 1 - ΕΡΩΤΗΜΑ 2
clear;
clf;
close all;
clc;

num_plant = 111;
den_plant = [1 11;
G = tf (num_plant, den_plant) ;
K_values = 1, 5, 10,50;
t = 0:0.01:5;
figure;
hold on;
grid on;
for k = K_values
  C = k;
  T = feedback(C * G, 1);
  [y, t_out] = step(T, t);
  error = 1 / (1 + k);
  plot(t_out, y, 'LineWidth', 1.5, ...
  'DisplayName', sprintf('Kp'= 8d, Epáxua = 8.2f', k, error));
end
y_ref = ones(size(t));
plot, y_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Στόχος (Reference)');
xlabel('Χρόνος(sec)');
ylabel ('Ταχύτητας v(t)');
legend ('Location', 'southeast');
hold off;

% ΜΕΡΟΣ 1 - ΕΡΩΤΗΜΑ 3

clear;
clf;
close all;
clc;

G = tf([1], [1,1]);
Kp_only = 5;
C_p = Kp_only;
T_p = feedback(C_p * G, 1);
Kp = 1;
Ki = 1;
C_pi = pid(Kp, Ki);
T_pi = feedback(C_pi * G, 1);
t = 0:0.01:10;
[y_p, t_p] = step(T_p, t);
[y_pi, t_pi] = step(T_pi, t);
figure;
hold on;
grid on;
y_ref=ones(size(t));
plot(t, y_ref, 'k--', 'LinedWidth', 1.5, 'DisplayName', 'Στόχος (Reference = 1)');
plot(t_p, y_p, 'r--', 'LinedWidth', 1.5, 'DisplayName', 'P-Controller (kp = 5)');
plot(t_pi, y_pi, 'b', 'LineWidth', 2, 'DisplayName', 'PI-Controller (Kp=1, Ki=1)');
title('Σύγκριση P vs PI: Εξάλειψη Σφάλματος');
xlabel('Χρόνος(sec)');
ylabel ('Ταχύτητας');
legend ('Location', 'southeast');

% ΜΕΡΟΣ 1 - ΕΡΩΤΗΜΑ 4

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s + 1);
Kp = 200;
Ga = 1 / (1 + s/100)^2;
L_actuator = Kp * Ga * G;
T_actuator = feedback(L_actuator, 1);
G_delay = G;
G_delay.InputDelay = 0.01;
L_delay = Kp * G_delay;
T_delay = feedback(L_delay, 1);
t = 0:0.001:2;
figure;
hold on;
grid on;
T_ideal = feedback(Kp * G, 1);
step(T_ideal, t, 'g--');
step(T_actuator, t, 'b');
step(T_delay, t, 'r');
legend('Ιδανικό Kp=200', ...
       'Με επενεργητή (Ga)', ...
       'Με Καθυστέρηση (0.01s)');
title('Επίδραση Υψηλού Κέρδους σε Μη-Ιδανικά Συστήματα');
xlabel('Χρόνος(sec)');
ylabel ('Ταχύτητα');
ylim([0 2.5]);

% ΜΕΡΟΣ 1 - ΕΡΩΤΗΜΑ 5Α

clear;
clf;
close all;
clc;

a_nom = 1;
b_nom = 1;
r = 1;
kr = a_nom / b_nom; 
fprintf('Υπολογισμένο κέρδος Feedforward (kr): %.2f\n', kr);
a_real = 1.1; 
b_real = 0.9;
d_disturbance = 0.10 * (kr * r);
u_control = kr * r;
total_input = (b_real * u_control) + d_disturbance;
s = tf('s');
G_sys = 1 / (s + a_real);
t = 0:0.01:10;
[v, t_out] = step(total_input * G_sys, t);
figure;
plot(t_out, v, 'r', 'LineWidth', 2); hold on;
yline(r, 'k--', 'LineWidth', 1.5); % Γραμμή στόχου
grid on;
title('Άσκηση 5Α: Απόκριση με Feedforward μόνο');
xlabel('Χρόνος (sec)');
ylabel('Ταχύτητα v(t)');
legend('Πραγματική Ταχύτητα', 'Στόχος (Reference)', 'Location', 'SouthEast');
final_v = v(end);
error = r - final_v;
fprintf('Τελική Ταχύτητα: %.4f\n', final_v);
fprintf('Σφάλμα μόνιμης κατάστασης: %.4f\n', error);

% ΜΕΡΟΣ 1 - ΕΡΩΤΗΜΑ 5Β

clear;
clf;
close all;
clc;

a_nom = 1;
b_nom = 1;
r = 1; 
kr = a_nom / b_nom;
a_real = 1.1; 
b_real = 0.9;
d_disturbance = 0.10 * (kr * r);
kp = 5;
a_closed = a_real + (b_real * kp);
total_input_closed = (b_real * (kr + kp) * r) + d_disturbance;
s = tf('s');
G_closed = 1 / (s + a_closed);
t = 0:0.01:10;
[v_fb, t_out] = step(total_input_closed * G_closed, t);
figure;
plot(t_out, v_fb, 'b', 'LineWidth', 2); hold on;
yline(r, 'k--', 'LineWidth', 1.5);
grid on;
title(['Άσκηση 5Β: Feedforward + Feedback (kp = ' num2str(kp) ')']);
xlabel('Χρόνος (sec)');
ylabel('Ταχύτητα v(t)');
legend('Πραγματική Ταχύτητα', 'Στόχος (Reference)', 'Location', 'SouthEast');
final_v = v_fb(end);
error = r - final_v;
fprintf('Τελική Ταχύτητα (με Feedback):
fprintf('Σφάλμα μόνιμης κατάστασης:

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 1

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s*(s+1));
K = [1, 5, 50];
figure;
hold on;
grid on;
for i = 1:length(K)
   T = feedback(K(i)*G, 1);
   step(T);
end
legend('K=1', 'K=5', 'K=50');

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 2

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s * (s + 1));
figure;
grid on;
hold on;
rlocus(G);
xline(-0.5, 'r--', 'LineWidth', 2, 'DisplayName', 'Real Part = -0.5');
title('Γεωμετρικός Τόπος Ριζών (Root Locus)');
legend('Root Locus', 'Σταθερό Πραγματικό Μέρος');
xlim([-3 1]);
ylim([-5 5]);

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 3 (Σύγκριση Αποκρίσεων)

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s^2 + s);
Zero_loc = 2; 
C_pd_structure = s + Zero_loc; 
figure;
subplot(1,2,1);
rlocus(G);
title('ΓΤΡ με P-Control (Παλιό)');
axis([-5 1 -3 3]); grid on;
xline(-0.5, 'r--', 'DisplayName', 'Όριο ταχύτητας');
subplot(1,2,2);
rlocus(C_pd_structure * G); 
title('ΓΤΡ με PD-Control (Νέο)');
axis([-5 1 -3 3]); grid on;
hold on; plot(-Zero_loc, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
legend('Root Locus', 'Zero at -2');
figure;
hold on; grid on;
K = 10;
T_p = feedback(K * G, 1);
% C(s) = 10 * (s + 2) = 10s + 20
T_pd = feedback(K * C_pd_structure * G, 1);
t = 0:0.01:10;
[y_p, t_p]   = step(T_p, t);
[y_pd, t_pd] = step(T_pd, t);
plot(t_p, y_p, 'r--', 'LineWidth', 1.5, 'DisplayName', 'P-Control Only');
plot(t_pd, y_pd, 'b', 'LineWidth', 2, 'DisplayName', 'PD-Control');
yline(1, 'k:');
title('Σύγκριση Απόκρισης: P vs PD');
xlabel('Χρόνος (sec)');
ylabel('Θέση');
legend('Location', 'Southeast');

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 3 (ΓΤΡ)

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s^2 + s);
z = 2;
G_pd = G * (s + z);
figure;
rlocus(G_pd);
hold on;
grid on;
plot(-z, 0, 'r o', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Zero (PD)');
title('Γεωμετρικός Τόπος Ριζών με PD Ελεγκτή');
legend('Location', 'best');
xlim([-6 2]);
ylim([-3 3]);

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 4

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s^2 + s);
Kp = 20;
Kd = 10;
C_pd = Kd*s + Kp;
T_dist = feedback(G, C_pd);
figure;
subplot(2,1,1);
step(T_dist);
grid on;
title('Απόκριση σε Βηματική Διαταραχή (d=1)');
ylabel('Θέση x(t)');
xlabel('Χρόνος (sec)');
expected_error = 1/Kp;
yline(expected_error, 'r--', 'LineWidth', 1.5);
legend('Απόκριση Συστήματος', ['Θεωρητικό Σφάλμα (1/Kp = ' num2str(expected_error) ')']);
subplot(2,1,2);
bode(T_dist);
grid on;
title('Διάγραμμα Bode (Disturbance to Output)');

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 5

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s^2 + s);
Kp = 20; 
Kd = 10;
C_pd = Kp + Kd*s;
Ki = 10; 
C_pid = Kp + Kd*s + Ki/s;
T_dist_PD  = feedback(G, C_pd);
T_dist_PID = feedback(G, C_pid);
figure;
subplot(2,1,1);
hold on; grid on;
step(T_dist_PD, 'r--');
step(T_dist_PID, 'b');
legend('PD (Μόνιμο Σφάλμα)', 'PID (Μηδενικό Σφάλμα)');
title('Απόκριση σε Σταθερή Διαταραχή (d=1)');
ylabel('Θέση x(t)');
subplot(2,1,2);
hold on; grid on;
bode(T_dist_PD, 'r--');
bode(T_dist_PID, 'b');
legend('PD', 'PID');
title('Bode Plot: Disturbance Rejection');

% ΜΕΡΟΣ 2 - ΕΡΩΤΗΜΑ 6

clear;
clf;
close all;
clc;

s = tf('s');
G = 1 / (s^2 + s);
Kp = 20; 
Kd = 10;
Ki = 5;
C_pid = Kp + Kd*s + Ki/s;
T_ref = feedback(C_pid * G, 1);
figure;
bode(T_ref);
grid on;
title('Διάγραμμα Bode Κλειστού Βρόχου (Tracking Capability)');
[bw, ~] = bandwidth(T_ref);
disp(['Το εύρος ζώνης (Bandwidth) είναι περίπου: ', num2str(bw), ' rad/s']);
t = 0:0.01:10;
w_low = 0.5; % rad/s
r_low = sin(w_low * t);
[y_low, ~] = lsim(T_ref, r_low, t);
w_high = 20; % rad/s (Πολύ γρήγορο σήμα)
r_high = sin(w_high * t);
[y_high, ~] = lsim(T_ref, r_high, t);
figure;
subplot(2,1,1);
plot(t, r_low, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_low, 'b', 'LineWidth', 2);
title(['Χαμηλή Συχνότητα (\omega = ', num2str(w_low), ' rad/s): Τέλεια Παρακολούθηση']);
legend('Είσοδος (Reference)', 'Έξοδος (Output)');
grid on;
subplot(2,1,2);
plot(t, r_high, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_high, 'r', 'LineWidth', 2);
title(['Υψηλή Συχνότητα (\omega = ', num2str(w_high), ' rad/s): Αδυναμία Παρακολούθησης']);
legend('Είσοδος', 'Έξοδος');
grid on;
