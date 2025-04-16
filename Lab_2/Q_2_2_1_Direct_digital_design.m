clear; clc;
run('load_params_inertial_case.m');

%% Step 2: Continuous-Time Plant Model
T = 0.01;  % Sample time in seconds (10 ms)
R_eq = mot.Ra + mot.Rs;
J_eq = 6.0087e-7;
B_eq = 6.6699e-7;
kdrv = drv.dcgain;

A = [0 1;
     0 -(R_eq*B_eq + mot.Kt*mot.Ke)/(R_eq*J_eq)];

B = [0;
     (kdrv * mot.Kt)/(gbox.N * R_eq * J_eq)];

C = [1 0];
D = 0;
sysC = ss(A,B,C,D);