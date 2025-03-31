%% System Constraints
t_s_5percent = 0.14;    % 5% Settling Time
M_p = 0.1;              % Overshoot


%% DC gear motor (plant) nominal parameters
kdrv = 0.6;             % Voltage driver static gain
fc_drv = 1200;          % Voltage driver cut-off frequency (Hz)
R_eq = mot.Ra + mot.Rs;	% Equivalent resistance  

%% Define Low-Pass Filter Parameters
w_ci = 2 * pi * 20;		% Cutoff frequency in rad/s
delta_i = 1 / sqrt(2);	% Damping coefficient

%% Define Butterworth Filter Parameters
w_c = 2 * pi * 20;		% Cutoff frequency in rad/s
delta_c = 1 / sqrt(2);	% Damping factor

%% Estimated Parameters from Lab_0
J_eq = 6.0087e-7;		% Estimated Equivalent Inertia
B_eq = 6.6699e-7;		% Estimated Equivalent Friction

%% State-Space Matrices

A = [ 0, 1; 0, - (R_eq*B_eq + mot.Kt*mot.Ke)/(R_eq*J_eq)];	% A matrix (System Matrix)
B = [ 0; (kdrv * mot.Kt)/(gbox.N * R_eq * J_eq)];			% B matrix (Input Matrix)
C = [1, 0];													% C matrix (Output Matrix)
D = 0;														% D matrix (Feedthrough/Direct Transmission Matrix)

%% Calculations Desired Poles

delta = log(1/M_p) / sqrt(pi^2 + log(1/M_p)^2);	% damping ratio from overshoot spec
wn = 3 / (delta * t_s_5percent);				% natural frequency from settling time spec

real_part = -delta * wn;				% real part of desired closed-loop poles
imag_part = wn * sqrt(1 - delta^2);		% imaginary part of desired closed-loop poles
poles = [real_part + 1i * imag_part, 
	real_part - 1i * imag_part];		% complex-conjugate pair of poles

K = place(A, B, poles);	% State feedback gain

M = [A, B; C, D];		% Block Matrix for Reference Tracking (3x3 Matrix)

rhs = [0; 0; 1];        % Right-hand side
sol = M \ rhs;          % Solve the system

Nx = sol(1:2);          % Steady-state state vector
Nu = sol(3);            % Steady-state input

Nr = Nu + K * Nx;		% Feedforward gain Nr

%% Print Results
fprintf('-------------------------------------------------\n');
fprintf('--- Desired Poles ---\n');
fprintf('-------------------------------------------------\n');
fprintf('λ1 = %.4f + %.4fj\n', real_part, imag_part); 
fprintf('λ2 = %.4f - %.4fj\n', real_part, imag_part);
fprintf('-------------------------------------------------\n');
fprintf('-------------------------------------------------\n');
fprintf('--- Augmented Matrix M = [A B; C 0] ---\n');
fprintf('-------------------------------------------------\n');
disp(M);
fprintf('-------------------------------------------------\n');
fprintf('-------------------------------------------------\n');
fprintf('--- Steady-State Feedforward Gains ---\n');
fprintf('-------------------------------------------------\n');
fprintf('Nx = [%.6e; %.6e]\n', Nx(1), Nx(2));
fprintf('Nu = %.6e\n', Nu);
fprintf('Nr = %.6e\n', Nr);
fprintf('-------------------------------------------------\n');
