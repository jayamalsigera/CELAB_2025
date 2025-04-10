load_params_inertial_case;
SS_Calculations;

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
r = -wn;

% Poles
lambda_1 = r * exp(1j * gamma_1);
lambda_2 = conj(lambda_1);

lambda_3 = r * exp(1j * gamma_2);
lambda_4 = conj(lambda_3);

lambda_5 = 2 * r;  % Real pole 

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
