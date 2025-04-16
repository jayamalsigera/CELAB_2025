%% Step 1: Discretize the Plant (2.2.1)
clear; clc;
load_params_inertial_case;  % Run this script, don't use load()

% Sampling time
T = 0.01; % 10 ms

% Motor model parameters
R_eq = mot.Ra + mot.Rs;
J_eq = 6.0087e-7;
B_eq = 6.6699e-7;
kdrv = drv.dcgain;

% Continuous-time state-space model
A = [0 1;
     0 -(R_eq*B_eq + mot.Kt*mot.Ke)/(R_eq*J_eq)];
B = [0;
     (kdrv * mot.Kt)/(gbox.N * R_eq * J_eq)];
C = [1 0];
D = 0;

sysC = ss(A, B, C, D);

% Discretize using ZOH
sysD = c2d(sysC, T, 'zoh');
[Phi, Gamma, H, J] = ssdata(sysD);

% Display
disp('Phi = '); disp(Phi);
disp('Gamma = '); disp(Gamma);
disp('H = '); disp(H);
