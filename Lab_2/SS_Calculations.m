load_params_inertial_case;
format long e; 
%% System Constraints
t_s_5percent = 0.15;    % 5% Settling Time
M_p = 0.1;              % Overshoot


%% DC gear motor (plant) nominal parameters
kdrv = drv.dcgain;             % Voltage driver static gain
fc_drv = 1200;          % Voltage driver cut-off frequency (Hz)
R_eq = mot.Ra + mot.Rs;	% Equivalent resistance  

%% Define Low-Pass Filter Parameters
w_ci = 2 * pi * 20;		% Cutoff frequency in rad/s
delta_i = 1 / sqrt(2);	% Damping coefficient

%% Define Butterworth Filter Parameters
w_c = 2 * pi * 50;		% Cutoff frequency in rad/s
delta_c = 1 / sqrt(2);	% Damping factor

%% Estimated Parameters from Lab_0
J_eq = 6.0087e-7;		% Estimated Equivalent Inertia
B_eq = 6.6699e-7;		% Estimated Equivalent Friction

%% State-Space Matrices

A = [ 0, 1; 
	0, -(R_eq*B_eq + mot.Kt*mot.Ke)/(R_eq*J_eq)];			% A matrix (System Matrix)
B = [ 0;
	(kdrv * mot.Kt)/(gbox.N * R_eq * J_eq)];				% B matrix (Input Matrix)
C = [1, 0];													% C matrix (Output Matrix)
D = 0;														% D matrix (Feedthrough/Direct Transmission Matrix)

delta = log(1/M_p) / sqrt(pi^2 + log(1/M_p)^2);	% damping ratio from overshoot spec
wn = 3 / (delta * t_s_5percent);				% natural frequency from settling time spec

real_part = -delta * wn;				% real part of desired closed-loop poles
imag_part = wn * sqrt(1 - delta^2);		% imaginary part of desired closed-loop poles