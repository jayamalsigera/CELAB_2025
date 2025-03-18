%% System Constraints
t_s_5percent = 0.15;    % 5% Settling Time
M_p = 0.1;              % Overshoot

%% DC gear motor (plant) nominal parameters
kdrv = 0.6;             % Voltage driver static gain
fc_drv = 1200;          % Voltage driver cut-off frequency (Hz)

%% Initial Assumptions
B_eq = 0;               % Equivalent damping (given as 0)
alpha = 15; % >=4       % T_I / T_D ratio
%T_j0 = ??;             % Assuming unity static gain

%% Equivalent Parameters

J_eq = mot.J + (3*gbox.J72 + mld.J) / gbox.N^2;     % Equivalent inertia
R_eq = mot.Ra + mot.Rs;                             % Equivalent resistance

%% Compute System Characteristics
zeta = log(1/M_p) / sqrt(pi^2 + (log(1/M_p))^2);    % Damping ratio from overshoot
w_n = 3 / (zeta * t_s_5percent);                    % Natural frequency (wn)
w_gc = 3 / (zeta * t_s_5percent);                   % Gain crossover frequency (w_gc)
t_r = 1.8 / w_gc;                                   % Rise time (tr)
t_s_1percent = 4.6 / (zeta * w_gc);                  % Settling time (1%) (ts_1percent)
phi_m_rad = atan((2 * zeta) / ...
    sqrt(sqrt(1 + 4 * zeta^4) - 2 * zeta^2));       % Phase margin (phi_m) in radians
phi_m_deg = rad2deg*phi_m_rad;                      % Phase margin to degrees
%M_r = (M_p + 1) * T_j0;                            % Resonant peak (Mr)
T_L = 2 * w_gc;										% Filter time constant (T_L)  {2-5 mot.Range}

%% Laplace Domain Calculations

k_m = (kdrv * mot.Kt) / (R_eq * B_eq + mot.Kt * mot.Ke);    % Gain k_m
T_m = (R_eq * J_eq) / (R_eq * B_eq + mot.Kt * mot.Ke);      % Constant T_m

s = 1i * w_gc;                                              % Define Laplace variable s
P_jwgc = (k_m / (s * T_m + 1)) * (1 / (gbox.N * s));        % Plant transfer function at gain crossover frequency

%% Compute DeltaK and DeltaPhi
DeltaK = 1 / abs(P_jwgc);                     % Gain change
DeltaPhi = -pi + phi_m_rad - angle(P_jwgc);   % Phase shift

%% Compute PID gains
K_P = DeltaK * cos(DeltaPhi);                           % Proportional gain
T_D = (tan(DeltaPhi) + sqrt((tan(DeltaPhi))^2 ...
	+ (4/alpha))) / (2*w_gc);							% Derivative time
T_I = alpha * T_D;                                      % Integral time
K_D = K_P * T_D;                                        % Derivative gain
K_I = K_P / T_I;                                        % Integral gain

J_eq = Jm + (3*J72 + Jd) / N^2;                                           % Equivalent inertia
R_eq = Ra + Rs;                                                         % Equivalent resistance

%% Compute System Characteristics
zeta = log(1/M_p) / sqrt(pi^2 + (log(1/M_p))^2);                        % Damping ratio from overshoot
w_n = 3 / (zeta * t_s_5percent);                                        % Natural frequency (wn)
w_gc = 3 / (zeta * t_s_5percent);                                       % Gain crossover frequency (w_gc)
t_r = 1.8 / w_gc;                                                        % Rise time (tr)
t_s_1percent = 4.6 / (zeta * w_gc);                                      % Settling time (1%) (ts_1percent)
phi_m_rad = atan((2 * zeta) / sqrt(sqrt(1 + 4 * zeta^4) - 2 * zeta^2)); % Phase margin (phi_m) in radians
phi_m_deg = rad2deg*phi_m_rad;                                         % Phase margin to degrees
M_r = (M_p + 1) * T_j0;                                                 % Resonant peak (Mr)
T_L = 1 / ((2/5) * w_gc);                                               % Filter time constant (T_L)

%% Laplace Domain Calculations
sx = 1j * w_gc;                                                          % Define Laplace variable at gain crossover frequency

k_m = (kdrv * kt) / (R_eq * B_eq + kt * ke);                            % Gain k_m
T_m = (R_eq * J_eq) / (R_eq * B_eq + kt * ke);                          % Constant T_m
P_jwgc = (k_m / (sx*T_m + 1)) * (1 / (N * sx));                           % Plant response at gain crossover frequency

%% Compute DeltaK and DeltaPhi
DeltaK = 1 / abs(P_jwgc);                                                                                                  % Gain change
DeltaPhi = -pi + phi_m_rad - angle(P_jwgc);                             % Phase shift

%% Compute PID gains
K_P = DeltaK * cos(DeltaPhi);                                           % Proportional gain
T_D = (tan(DeltaPhi) + sqrt((tan(DeltaPhi))^2 + (4/alpha))) / (2*w_gc); % Derivative time
T_I = alpha * T_D;                                                      % Integral time
K_D = K_P * T_D;                                                        % Derivative gain
K_I = K_P / T_I;                                                        % Integral gain

%% Display results
fprintf('Computed System Parameters:\n');
fprintf('--------------------------------------\n');
fprintf('Damping mot.Ratio (zeta): %.6f\n', zeta);
fprintf('Natural Frequency (w_n): %.6f rad/s\n', w_n);
fprintf('Gain Crossover Frequency (w_gc): %.6f rad/s\n', w_gc);
fprintf('Rise Time (tr): %.6f s\n', t_r);
fprintf('Settling Time (5%%) (t_s_5percent): %.6f s\n', t_s_5percent);
fprintf('Settling Time (1%%) (t_s_1percent): %.6f s\n', t_s_1percent);
fprintf('Phase Margin (phi_m): %.6f degrees\n', phi_m_deg);
%fprintf('Resonant Peak (M_r): %.6f\n', M_r);
fprintf('Filter Time Constant (T_L): %.6f s\n', T_L);
fprintf('--------------------------------------\n');
fprintf('Computed Controller Gains:\n');
fprintf('--------------------------------------\n');
fprintf('K_P = %.6f\n', K_P);
fprintf('K_D = %.6f\n', K_D);
fprintf('K_I = %.6f\n', K_I);
