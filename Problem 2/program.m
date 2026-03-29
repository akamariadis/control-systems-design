clear;
close all;
clf;
clc;

A = [-2, -0.02; 1, -10];
B = [2; 0];
C = [0, 1];
D = 0;
sys_open = ss(A, B, C, D);
P = [-10, -15]; 
K = place(A, B, P);
disp('Ο πίνακας κερδών K είναι:');
disp(K);
A_cl = A - B*K;
kr = -1 / (C * inv(A_cl) * B);
disp('Το κέρδος kr είναι:');
disp(kr);
B_cl = B * kr;
sys_closed = ss(A_cl, B_cl, C, D);
t = 0:0.01:2;
step(sys_closed, t);
title('Απόκριση Ταχύτητας Κινητήρα (Κλειστός Βρόχος) σε r=1');
xlabel('Χρόνος (sec)');
ylabel('Ταχύτητα \omega (rad/s)');
grid on;

clear;
close all;
clf;
clc;

A_cl = [-15, 0; 1, -10];
Bd = [0; 100];
C = [0, 1];
Dd = 0;
sys_disturbance = ss(A_cl, Bd, C, Dd);
disp('Συνάρτηση Μεταφοράς από d σε y:');
tf(sys_disturbance)
figure;
bode(sys_disturbance);
grid on;
title('Διάγραμμα Bode Κλειστού Βρόχου (Διαταραχή -> Έξοδος)');

clear;
close all;
clf;
clc;

A = [-2, -0.02; 1, -10];
B = [2; 0];
C = [0, 1];
Bd = [0; 100];

A_aug = [A, zeros(2,1); -C, 0];
B_aug = [B; 0];
Bd_aug = [Bd; 0];
C_aug = [C, 0];
D_aug = 0;

P = [-10, -15, -20]; 
K_aug = place(A_aug, B_aug, P);

A_cl = A_aug - B_aug * K_aug;
sys_dist_int = ss(A_cl, Bd_aug, C_aug, D_aug);

disp('Συνάρτηση Μεταφοράς από d σε y (με ολοκληρωτή):');
tf(sys_dist_int)

figure;
bode(sys_dist_int);
grid on;
title('Bode Διαταραχής -> Εξόδου (Με Ολοκληρωτή)');
