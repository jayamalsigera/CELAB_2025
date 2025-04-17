%% Compute DeltaK and DeltaPhi
DeltaK = 1/abs(P_jwgc);                     % Gain change
DeltaPhi = -pi + phi_m_rad - angle(P_jwgc);   % Phase shift

%% Compute PID gains for w
K_P = DeltaK * cos(DeltaPhi);                           % Proportional gain
T_D = (tan(DeltaPhi) + sqrt((tan(DeltaPhi))^2 ...
	+ (4/alpha))) / (2*w_gc);							% Derivative time
T_I = alpha * T_D;                                      % Integral time
K_D = K_P * T_D;                                        % Derivative gain
K_I = K_P / T_I;                                        % Integral gain

%% Display results
fprintf('===========================================================\n');
fprintf('PID Calculations\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Computed System Parameters:\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Damping Ratio (zeta): %.6f\n', zeta);
fprintf('Natural Frequency (w_n): %.6f rad/s\n', w_n);
fprintf('Gain Crossover Frequency (w_gc): %.6f rad/s\n', w_gc);
fprintf('Rise Time (tr): %.6f s\n', t_r);
fprintf('Settling Time (5%%) (t_s_5percent): %.6f s\n', t_s_5percent);
fprintf('Settling Time (1%%) (t_s_1percent): %.6f s\n', t_s_1percent);
fprintf('Phase Margin (phi_m): %.6f degrees\n', phi_m_deg);
%fprintf('Resonant Peak (M_r): %.6f\n', M_r);
fprintf('Filter Time Constant (T_L): %.6f s\n', T_L);
fprintf('-----------------------------------------------------------\n');
fprintf('Computed Controller Gains:\n');
fprintf('-----------------------------------------------------------\n');
fprintf('K_P = %.6f\n', K_P);
fprintf('K_D = %.6f\n', K_D);
fprintf('K_I = %.6f\n', K_I);
fprintf('K_W = %.6f\n', K_W);
fprintf('-----------------------------------------------------------\n');