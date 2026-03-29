% SIMULINK

clear;
close all;
clf;
clc;

A = [-2, -0.02; 1, -10];
B = [2; 0];
C = [0, 1];
D = 0;

K = [6.5, -0.01]; 
kr = 75; 
L = [1064; 58];

A_ctrl = A - B*K - L*C;
B_ctrl = [L, B*kr];
C_ctrl = -K;
D_ctrl = [0, kr];

A_closed = [A, -B*K; L*C, A - B*K - L*C];
B_closed = [B*kr; B*kr];
C_closed = [C, zeros(1,2)];
D_closed = 0;

sys_total = ss(A_closed, B_closed, C_closed, D_closed);

step(sys_total);
title('Απόκριση Ταχύτητας με Παρατηρητή');
grid on;