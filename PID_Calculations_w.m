%% Initial Assumptions
B_eq_w = 0;					% Equivalent damping (given as 0)
alpha_w = 4; % >=4			% T_I / T_D ratio

%% System Constraints
t_s_5percent_w = 0.15;    % 5% Settling Time
M_p_w = 0.1;              % Overshoot

%% Compute System Characteristics
zeta_w = log(1/M_p_w) / sqrt(pi^2 + (log(1/M_p_w))^2);		% Damping ratio from overshoot
w_n_w = 3 / (zeta_w * t_s_5percent_w);						% Natural frequency (wn)
w_gc_w = 3 / (zeta_w * t_s_5percent_w);						% Gain crossover frequency (w_gc)
t_r_w = 1.8 / w_gc_w;										% Rise time (tr)
t_s_1percent_w = 4.6 / (zeta_w * w_gc_w);					% Settling time (1%) (ts_1percent)
phi_m_rad_w = atan((2 * zeta_w) / ...
    sqrt(sqrt(1 + 4 * zeta_w^4) - 2 * zeta_w^2));			% Phase margin (phi_m) in radians
phi_m_deg_w = rad2deg * phi_m_rad_w;							% Phase margin to degrees
T_L_w = 5 * w_gc_w;											% Filter time constant (T_L)  {2-5 mot.Range}

%% Define Butterworth High-Pass Filter Parameters
w_c = 2 * pi * 20;			% Cutoff frequency in rad/s
delta_c = 1 / sqrt(2);		% Damping factor for Butterworth filter

%% Laplace Domain Calculations

% Define motor parameters
k_m_w = (kdrv * mot.Kt) / (R_eq * B_eq + mot.Kt * mot.Ke);    % Gain k_m
T_m_w = (R_eq * J_eq) / (R_eq * B_eq + mot.Kt * mot.Ke);      % Constant T_m

s_w = 1i * w_gc_w;			% Define Laplace variable s at crossover frequency

P_jwgc_ = (k_m / (s_w * T_m + 1)) * (1 / (gbox.N * s_w));        % Plant transfer function at gain crossover frequency

% Compute plant transfer function at omega_gc
H_omega = (w_c^2 * s_w) / (s_w^2 + 2 * delta_c * w_c * s_w + w_c^2); % Butterworth Filter

P_jwgc_w = P_jwgc_ * s_w;

%% Compute Gain Margin and Phase Margin
DeltaK_w = 1 / abs(P_jwgc_w);						% Gain margin shift
DeltaPhi_w = -pi + phi_m_rad_w - angle(P_jwgc_w);	% Phase margin shift

%% Compute PID Gains
K_P_w = DeltaK_w * cos(DeltaPhi_w);                          % Proportional gain
T_D_w = (tan(DeltaPhi_w) + sqrt((tan(DeltaPhi_w))^2 + (4/alpha_w))) / (2 * w_gc_w);  % Derivative time
T_I_w = alpha_w * T_D_w;                                       % Integral time
K_D_w = K_P_w * T_D_w;                                       % Derivative gain
K_I_w = K_P_w / T_I_w;                                       % Integral gain

%% Print Results
fprintf('Gain Margin Shift = %.6f\n', DeltaK_w);
fprintf('Phase Margin Shift = %.6f\n', rad2deg*DeltaPhi_w);
fprintf('--------------------------------------\n');
fprintf('Computed Controller Gains:\n');
fprintf('--------------------------------------\n');
fprintf('K_P = %.6f\n', K_P_w);
fprintf('K_D = %.6f\n', K_D_w);
fprintf('K_I = %.6f\n', K_I_w);
