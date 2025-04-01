load_params_inertial_case;

%% System Constraints
t_s_5percent = 0.15;    % 5% Settling Time
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

A = [ 0, 1; 0, -(R_eq*B_eq + mot.Kt*mot.Ke)/(R_eq*J_eq)];	% A matrix (System Matrix)
B = [ 0; (kdrv * mot.Kt)/(gbox.N * R_eq * J_eq)];			% B matrix (Input Matrix)
C = [1, 0];													% C matrix (Output Matrix)
D = 0;														% D matrix (Feedthrough/Direct Transmission Matrix)

%%  Nominal tracking design

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
fprintf('---Nominal Tracking Design---\n');
fprintf('-------------------------------------------------\n');
fprintf('--- Desired Poles ---\n');
fprintf('λ1 = %.4f + %.4fj\n', real_part, imag_part); 
fprintf('λ2 = %.4f - %.4fj\n', real_part, imag_part);
fprintf('\n--- Augmented Matrix M = [A B; C 0] ---\n');
disp(M);
fprintf('\n--- State Feedback Gains ---\n');
fprintf('K = [%.6e; %.6e]\n', K(1), K(2));
fprintf('\n--- State Feedforward Gains ---\n');
fprintf('Nx = [%.6e; %.6e]\n', Nx(1), Nx(2));
fprintf('Nu = %.6e\n', Nu);
fprintf('Nr = %.6e\n', Nr);
fprintf('-------------------------------------------------\n');

%% Robust tracking design with integral action

Ae = [0, C; zeros(size(A,1),1), A];     % 3x3 matrix
Be = [0; B];							% 3x1 input matrix
Ce = [0, C];							% 1x3 output matrix

% Place poles and extract gains
fprintf('\n\n-------------------------------------------------\n');
fprintf('---  Robust tracking design with integral action ---\n');
fprintf('-------------------------------------------------\n');

% Choose your desired pole set (e.g., p3)
poles = [2.5*real_part, 2*real_part, 0.05*real_part];

try
    % Place poles for augmented system
    Ke = place(Ae, Be, poles);

    % Extract gains
    KIe = Ke(1); 
    Kxe = Ke(2:3); 
    K1e = Kxe(1);
    K2e = Kxe(2);

    % Solve reference tracking equations
    sole = M \ rhs;
    Nxe = sol(1:2);
    Nxe_aug = [0; Nxe];               % Pad Nx to match augmented state
    Nue = sol(3);
    Nre = Nue + Kxe * Nxe;

    % Store results in base workspace (optional)
    assignin('base', 'Ke', Ke);
    assignin('base', 'KI', KIe);
    assignin('base', 'K1', K1e);
    assignin('base', 'K2', K2e);
    assignin('base', 'Nx', Nxe_aug);
    assignin('base', 'Nu', Nue);
    assignin('base', 'Nr', Nre);

    % Print results
    fprintf('\n--- Selected Pole Placement ---\n');
    fprintf('Ke = [%.4f %.4f %.4f]\n', KIe, K1e, K2e);
    fprintf('KI = %.4f\n', KI);
    fprintf('Nx = [%.4f; %.4f; %.4f]\n', Nxe_aug(1), Nxe_aug(2), Nxe_aug(3));
    fprintf('Nu = %.4f\n', Nue);
    fprintf('Nr = %.4f\n', Nre);

catch ME
    fprintf('\n--- Computation Skipped --- %s\n', ME.message);
end

Acl = Ae - Be * Ke;						% 3x3 closed-loop matrix