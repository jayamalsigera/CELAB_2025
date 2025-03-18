%% Laplace Domain Calculations

% Define motor parameters
k_m_w = (kdrv * mot.Kt) / (R_eq * B_eq + mot.Kt * mot.Ke);    % Gain k_m
T_m_w = (R_eq * J_eq) / (R_eq * B_eq + mot.Kt * mot.Ke);      % Constant T_m

s_w = 1i * w_gc;  % Define Laplace variable s at crossover frequency

% Compute plant transfer function at omega_gc
P_jwgc_w = (k_m_w / (s_w * T_m_w + 1)) * (1 / gbox.N);        

%% Compute Gain Margin and Phase Margin
DeltaK_w = 1 / abs(P_jwgc_w);                    % Gain margin shift
DeltaPhi_w = -pi + phi_m_rad - angle(P_jwgc_w);  % Phase margin shift

%% Compute PID Gains
K_P_w = DeltaK_w * cos(DeltaPhi_w);                          % Proportional gain
T_D_w = (tan(DeltaPhi_w) + sqrt((tan(DeltaPhi_w))^2 + (4/alpha))) / (2*w_gc);  % Derivative time
T_I_w = alpha * T_D_w;                                       % Integral time
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
