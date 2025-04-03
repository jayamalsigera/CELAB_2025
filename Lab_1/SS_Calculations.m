load_params_inertial_case;
format long e; 
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

%%  Nominal tracking design

delta = log(1/M_p) / sqrt(pi^2 + log(1/M_p)^2);	% damping ratio from overshoot spec
wn = 3 / (delta * t_s_5percent);				% natural frequency from settling time spec

real_part = -delta * wn;				% real part of desired closed-loop poles
imag_part = wn * sqrt(1 - delta^2);		% imaginary part of desired closed-loop poles
beta1 = real_part + 1i * imag_part;
beta2 = real_part - 1i * imag_part;
poles = [beta1, 
		beta2];		% complex-conjugate pair of poles

K = place(A, B, poles);	% State feedback gain

M = [A, B;
	C, D];				% Block Matrix for Reference Tracking (3x3 Matrix)

rhs = [0; 
	0; 
	1];					% Right-hand side
sol = M \ rhs;          % Solve the system (M * [Nx; Nu] = [0; 0; 1])

Nx = sol(1:2);          % State prefilter
Nu = sol(3);            % Input prefilter

Nr = Nu + K * Nx;		% Feedforward gain

%% Print Results
fprintf('===========================================================\n');
fprintf('---Nominal Tracking Design---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Desired Poles ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('λ1 = %.4f + %.4fj\n', real_part, imag_part); 
fprintf('λ2 = %.4f - %.4fj\n', real_part, imag_part);
fprintf('-----------------------------------------------------------\n');
fprintf('\n--- Augmented Matrix M = [A B; C 0] ---\n');
fprintf('-----------------------------------------------------------\n');
disp(M);
fprintf('-----------------------------------------------------------\n');
fprintf('--- State Feedback Gains ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('K = [%.6e; %.6e]\n', K(1), K(2));
fprintf('-----------------------------------------------------------\n');
fprintf('--- State Feedforward Gains ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Nx = [%.6e; %.6e]\n', Nx(1), Nx(2));
fprintf('Nu = %.6e\n', Nu);
fprintf('Nr = %.6e\n', Nr);
fprintf('===========================================================\n');

%% Robust tracking design with integral action

Ae = [0, C; zeros(size(A,1),1), A];     % 3x3 matrix
Be = [0; B];							% 3x1 input matrix
Ce = [0, C];							% 1x3 output matrix

fprintf('===========================================================\n');
fprintf('---  Robust tracking design with integral action ---\n');
fprintf('===========================================================\n');

% Pole placement (3 poles for 3x3 system)
poles = [2.5*real_part, 2*real_part, 0.05*real_part];

try
    % Pole placement for augmented system
    Ke = place(Ae, Be, poles);			% Design gain for augmented system

    % Extract gains
    KIe = Ke(1);						% Integral gain
    Kxe = Ke(2:3);						% State feedback gains
    K1e = Kxe(1);						% State gain 1
    K2e = Kxe(2);						% State gain 2

    % Solve reference tracking equations
    sole = M \ rhs;						% Solve the system (M * [Nx; Nu] = [0; 0; 1])
    Nxe = sol(1:2);						% State prefilter
    Nxe_aug = [0; Nxe];					% Change Nx to match augmented state
    Nue = sol(3);						% Input prefilter
    Nre = Nue + Kxe * Nxe;				% Feedforward gain for augmented system


    % Print results
	fprintf('-----------------------------------------------------------\n');
    fprintf('--- Selected Pole Placement ---\n');
	fprintf('-----------------------------------------------------------\n');
    fprintf('Ke = [%.4f %.4f %.4f]\n', KIe, K1e, K2e);
    fprintf('KI = %.4f\n', KI);
    fprintf('Nx = [%.4f; %.4f; %.4f]\n', Nxe_aug(1), Nxe_aug(2), Nxe_aug(3));
    fprintf('Nu = %.4f\n', Nue);
    fprintf('Nr = %.4f\n', Nre);
fprintf('-----------------------------------------------------------\n');
catch ME
    fprintf('\n--- Computation Skipped --- %s\n', ME.message);
end
fprintf('===========================================================\n');

Acl = Ae - Be * Ke;						% 3x3 closed-loop matrix of integral system

%% Q_2_2_SSC_Robust_Tracking_with_Error_Space (FOR SINUSIOID)

Tr = 0.5;								% Sinusoid reference signal period [s]
omega_0 = 2 * pi / Tr;					% Angular frequency (rad/s)
alpha_z = [1 0 omega_0^2 0];			% Polynomial for: s^3 + ω₀² s

m = length(alpha_z) - 1;				% Internal model order (m = 3)
n = size(A,1);							% Plant order

%Az Calculation (Equation 67: Lab 1 Handout)
%Top-left block
Az_top_left_top = [zeros(m-1,1), eye(m-1)];				% First m-1 rows of companion form
Az_top_left_bottom = -fliplr(alpha_z(2:end));			% Last row = -α coefficients
Az_top_left = [Az_top_left_top; Az_top_left_bottom];	% Full top-left block
%Top-right block
Az_top_right = [zeros(m-1, n); C];			%zeros except for last row is C
%Bottom blocks
Az_bottom_left = zeros(n, m);				% Plant not affected by error dynamics
Az_bottom_right = A;						% Original plant dynamics
%Combine
Az = [Az_top_left, Az_top_right;			% Top half	
         Az_bottom_left, Az_bottom_right];	% Bottom half

Az_num_rows = size(Az, 1);
disp(['Number of rows in Az: ', num2str(Az_num_rows)]);

% Bz Calculation (Equation 67: Lab 1 Handout)
Bz = [zeros(size(Az,1)-size(B,1), size(B,2)); B];

% Pole Phases
gamma_1 = 2.5*pi/6;			%5π/12
gamma_2 = 1.5*pi/4;			%3π/8

% Pole Magnitude
r = wn;

% Poles
lambda_1 = r * exp(1j * gamma_1);
lambda_2 = conj(lambda1);

lambda_3 = r * exp(1j * gamma_2);
lambda_4 = conj(lambda3);

lambda_5 = -2 * r;  % Real pole 

% Poles Combined
poles_z = [lambda_1, lambda_2, lambda_3, lambda_4, lambda_5];	% Poles


% Check Controllability
[mn, ~] = size(Az);

if rank(ctrb(Az, Bz)) < mn
    error('Az, Bz is not controllable');
end

% Gain Calculation
K_epsilon = place(Az, Bz, poles_z);		% Full state feedback gain for error-space system
kz_error = K_epsilon(1:m);				% Gains on error and its derivatives (e, e', e'')
K_xi     = K_epsilon(m+1:end);			% Gains on plant states (x)

H_num = fliplr(real(kz_error));			% Numerator coefficients
H_den = alpha_z;						% Denominator coefficients

H = tf(H_num, H_den);					% Define the compensator TF H(s)


fprintf('===========================================================\n');
fprintf('--- Internal Model (Sinusoid) ---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Error System Matrices ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Az =\n'); 
disp(Az);
fprintf('Bz =\n'); 
disp(Bz);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Error-Space Pole Placement ---\n');
fprintf('-----------------------------------------------------------\n');
for i = 1:length(poles_z)
    fprintf('λ_c,%d = %.4f %+.4fj\n', i, real(poles_z(i)), imag(poles_z(i)));
end
fprintf('-----------------------------------------------------------\n');
fprintf('--- Full Error-Space Feedback Gain ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('K_epsilon =\n'); 
disp(K_epsilon);
fprintf('-----------------------------------------------------------\n');
fprintf('kz_error =\n'); 
disp(kz_error);
fprintf('-----------------------------------------------------------\n');
fprintf('K_xi =\n'); 
disp(K_xi);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Compensator H(s) = U_tilde(s) / E(s) ---\n');
fprintf('-----------------------------------------------------------\n');
display(H);
disp(H);
fprintf('-----------------------------------------------------------\n');
fprintf('===========================================================');


%% Q_2_2_SSC_Robust_Tracking_with_Extended_State_Estimator (FOR SINUSIOID)


%Az Calculation (Equation 67: Lab 1 Handout)
%Top-left block
As_top_left_top = [zeros(m-1,1), eye(m-1)];				% First m-1 rows of companion form
As_top_left_bottom = -fliplr(alpha_z(2:m+1));			% Last row = -α coefficients
As_top_left = [Az_top_left_top; Az_top_left_bottom];	% Full top-left block
%Top-right block
As_top_right = [zeros(m, n)];				% zeros
%Bottom blocks
As_bottom_left = [B, zeros(n, m-1)];				% Plant not affected by error dynamics
As_bottom_right = A;						% Original plant dynamics
%Combine
As = [As_top_left, As_top_right;			% Top half	
         As_bottom_left, As_bottom_right];	% Bottom half

As_num_rows = size(As, 1);
disp(['Number of rows in As: ', num2str(As_num_rows)]);

Bs = [zeros(m,1);
       B];

Bs_num_rows = size(Bs, 1);
disp(['Number of rows in Bs: ', num2str(Bs_num_rows)]);

Cs = [zeros(1, m), C];

lambda_e1 = 15 * wn * exp(1j * (-pi + 0.1*pi/3));
lambda_e2 = conj(lambda_e1);

lambda_e3 = 15* wn * exp(1j * (-pi + 0.01*pi/4));
lambda_e4 = conj(lambda_e3);

lambda_e5 = -15 * wn;

poles_ee = [lambda_e1, lambda_e2, lambda_e3, lambda_e4, lambda_e5];

Ls = place(As', Cs', poles_ee)';

beta1e = real_part + 1i * imag_part;
beta2e = conj(beta1e);
poles_ce = [beta1e, 
		beta2e];		% complex-conjugate pair of poles

Kce = place(A, B, poles_ce);

fprintf('===========================================================\n');
fprintf('--- Extended Estimator Model (Sinusoid) ---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Extended Estimator System Matrices ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('As =\n'); 
disp(As);
fprintf('Bs =\n'); 
disp(Bs);
fprintf('Cs =\n'); 
disp(Cs);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Extended Estimator Pole Placement ---\n');
fprintf('-----------------------------------------------------------\n');
for i = 1:length(poles_ee)
    fprintf('λ_c,%d = %.4f %+.4fj\n', i, real(poles_ee(i)), imag(poles_ee(i)));
end
fprintf('-----------------------------------------------------------\n');
fprintf('--- Extended Estimator Pole PlacementObserver Gain Matrix ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Ls =\n');
disp(Ls);