load_params_inertial_case;

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

K0 = place(A, B, poles);	% State feedback gain

M = [A, B; C, D];		% Block Matrix for Reference Tracking (3x3 Matrix)

rhs = [0; 0; 1];        % Right-hand side
sol = M \ rhs;          % Solve the system

Nx0 = sol(1:2);          % Steady-state state vector
Nu0 = sol(3);            % Steady-state input

Nr0 = Nu0 + K0 * Nx0;		% Feedforward gain Nr

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
fprintf('K = [%.6e; %.6e]\n', K0(1), K0(2));
fprintf('\n--- State Feedforward Gains ---\n');
fprintf('Nx = [%.6e; %.6e]\n', Nx0(1), Nx0(2));
fprintf('Nu = %.6e\n', Nu0);
fprintf('Nr = %.6e\n', Nr0);
fprintf('-------------------------------------------------\n');

%% Robust tracking design with integral action

Ae = [0, C; zeros(size(A,1),1), A];     % 3x3 matrix
Be = [0; B];							% 3x1 input matrix

% Place poles and extract gains
fprintf('\n\n-------------------------------------------------\n');
fprintf('---  Robust tracking design with integral action ---\n');
fprintf('-------------------------------------------------\n');

% Define pole sets
p1 = [real_part + 1i*imag_part, real_part - 1i*imag_part, real_part];
p2 = [real_part, real_part, real_part];
p3 = [2*real_part + 1i*imag_part, 2*real_part - 1i*imag_part, 2*real_part];
p4 = [2*real_part + 1i*imag_part, 2*real_part - 1i*imag_part, 3*real_part];

pole_sets = {p1, p2, p3, p4};

% Loop over all pole sets
for i = 1:4
    try
        % Place poles for augmented system
        poles = pole_sets{i};
        Ke = place(Ae, Be, poles);

        % Extract gains
        KI = Ke(1); 
        Kx = Ke(2:3); 
        K1 = Kx(1);
        K2 = Kx(2);

        % Store gains in workspace with unique names
        assignin('base', sprintf('Ke%d', i), Ke);
        assignin('base', sprintf('KI%d', i), KI);
        assignin('base', sprintf('K1_%d', i), K1);
        assignin('base', sprintf('K2_%d', i), K2);

        % Solve reference tracking
        sol = M \ rhs;
        Nx = sol(1:2);
		Nx_aug = [0; Nx];
        Nu = sol(3);
        Nr = Nu + Kx * Nx;

        % Store tracking gains
        assignin('base', sprintf('Nx%d', i), Nx_aug);
        assignin('base', sprintf('Nu%d', i), Nu);
        assignin('base', sprintf('Nr%d', i), Nr);

        % Print results
        fprintf('\n--- Option %d ---\n', i);
        fprintf('Ke%d = [%.4f %.4f %.4f]\n', i, KI, K1, K2);
        fprintf('KI%d = %.4f\n', i, KI);
		fprintf('Nx%d = [%.4f; %.4f; %.4f]\n', i, Nx_aug(1), Nx_aug(2), Nx_aug(3) );
        fprintf('Nu%d = %.4f\n', i, Nu);
        fprintf('Nr%d = %.4f\n', i, Nr);

    catch ME
        fprintf('\n---Option %d--- Skipped — %s\n', i, ME.message);
    end
end