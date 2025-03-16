% Define given parameters
Ra = 2.6; % Armature resistance (Ohms)
La = 180e-6; % Armature inductance (H)
ke = 7.68e-3; % Electric (BEMF) constant (Vs/rad)
kt = 7.68e-3; % Torque constant (Nm/A)
Jm = 3.90e-7; % Rotor inertia (kg*m^2)
N = 14; % Gearbox ratio
J72 = 1.4e-6; % Moment of inertia of external gear (kg*m^2)
Jd = 3.0e-5; % Moment of inertia of extra disc (kg*m^2)
kdrv = 0.6; % Voltage driver static gain
fc_drv = 1200; % Voltage driver cut-off frequency (Hz)
Rs = 0.5; % Shunt resistance (Ohms)
B_eq = 0; % Equivalent damping (given as 0)
alpha = 4; % T_I / T_D ratio
phi_m_deg = 58.6; % Phase margin in degrees
phi_m = deg2rad(phi_m_deg); % Convert to radians

% Equivalent parameters
J_eq = Jm + (J72 + Jd) / N^2;
R_eq = Ra + Rs;

% Compute k_m and T_m
k_m = (kdrv * kt) / (R_eq * B_eq + kt * ke);
T_m = (R_eq * J_eq) / (R_eq * B_eq + kt * ke);

% Compute gain crossover frequency (previously calculated)
w_gc = 11.28;

% Compute damping ratio
Mp = 0.1;
zeta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);

% Compute natural frequency
omega_n = 3 / (zeta * 0.15);

% Compute rise time and settling times
t_r = 1.8 / omega_n;
t_s_1 = 4.6 / (zeta * omega_n);

% Compute filter cutoff frequency
T_L = 1 / (7/5 * w_gc);

% Compute P(jw_gc)
s = 1j * w_gc;
P_jwgc = (k_m / (s*T_m + 1)) * (1 / (N * s));

% Compute DeltaK and DeltaPhi
DeltaK = 1 / abs(P_jwgc);
DeltaPhi = -pi + phi_m - angle(P_jwgc);

% Compute PID gains
K_P = DeltaK * cos(DeltaPhi);
T_D = (tan(DeltaPhi) + sqrt(tan(DeltaPhi)^2 + 4/alpha)) / (2*w_gc);
T_I = alpha * T_D;
K_D = K_P * T_D;
K_I = K_P / T_I;

% Display results
fprintf('Damping Ratio (zeta) = %.4f\n', zeta);
fprintf('Natural Frequency (omega_n) = %.4f rad/s\n', omega_n);
fprintf('Gain Crossover Frequency (omega_gc) = %.4f rad/s\n', w_gc);
fprintf('Rise Time (t_r) = %.4f s\n', t_r);
fprintf('Settling Time 1%% (t_s_1) = %.4f s\n', t_s_1);
fprintf('Filter Time Constant (T_L) = %.4f s\n', T_L);
fprintf('K_P = %.4f\n', K_P);
fprintf('K_D = %.4f\n', K_D);
fprintf('K_I = %.4f\n', K_I);
