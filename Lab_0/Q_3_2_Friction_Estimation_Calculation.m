%% Load Filtered Data
% load data to workspace
load('filtered_positive.mat'); % Load filtered_positive dataset
load('filtered_negative.mat'); % Load filtered_negative dataset

% Extract data from filtered datasets
tau_m_plus = filtered_positive(:,1); % Torque for positive speed dataset
omega_m_plus_rpm = filtered_positive(:,2); % Speed (in RPM/s)

tau_m_minus = filtered_negative(:,1); % Torque for negative speed dataset
omega_m_minus_rpm = filtered_negative(:,2); % Speed (in RPM/s)

% Convert Speed from RPM/s to rad/s
omega_m_plus = omega_m_plus_rpm * (2 * pi / 60);
omega_m_minus = omega_m_minus_rpm * (2 * pi / 60);

% Define gear ratio N
N = 14;  

%% Ensure Data is Continuous (Column Vectors)
tau_m_plus = tau_m_plus(:); % Convert to column vector
omega_m_plus = omega_m_plus(:);
tau_m_minus = tau_m_minus(:);
omega_m_minus = omega_m_minus(:);

%% Create Regressor Matrix
% Construct regressor matrices for both positive and negative datasets
Phi_plus = [omega_m_plus, (1/N) * sign(omega_m_plus)];
Phi_minus = [omega_m_minus, (1/N) * sign(omega_m_minus)];

% Construct torque vector
Y_plus = tau_m_plus;
Y_minus = tau_m_minus;

%% Least Squares Estimation
theta_LS_plus = (Phi_plus' * Phi_plus) \ (Phi_plus' * Y_plus);
theta_LS_minus = (Phi_minus' * Phi_minus) \ (Phi_minus' * Y_minus);

% Extract estimated friction parameters
B_eq_plus = theta_LS_plus(1);
tau_sf_plus = theta_LS_plus(2);
B_eq_minus = theta_LS_minus(1);
tau_sf_minus = theta_LS_minus(2);

% Final friction parameter estimates
B_eq = (B_eq_plus + B_eq_minus) / 2;
tau_sf = (abs(tau_sf_plus) + abs(tau_sf_minus)) / 2;

%% Display Results
fprintf('------------------------------------------------\n');
%% Print Best-Fit Line Parameters
fprintf('Best-Fit Line Parameters:\n');
fprintf('------------------------------------------------\n');
fprintf('B_eq_plus   = %.4e [Nm/(rad/s)]\n', B_eq_plus);
fprintf('tau_sf_plus = %.4e [Nm]\n', tau_sf_plus);
fprintf('B_eq_minus  = %.4e [Nm/(rad/s)]\n', B_eq_minus);
fprintf('tau_sf_minus = %.4e [Nm]\n', tau_sf_minus);
fprintf('------------------------------------------------\n');
fprintf('Estimated Friction Parameters:\n');
fprintf('------------------------------------------------\n');
fprintf('B_eq   = %.4e [Nm/(rad/s)]\n', B_eq);
fprintf('tau_sf = %.4e [Nm]\n', tau_sf);
fprintf('------------------------------------------------\n');


%% Confidence Interval Calculation
M = length(tau_m_plus) + length(tau_m_minus); % Total measurements
p = 2; % Number of parameters estimated

% Least squares residuals
V_theta = norm([Y_plus; Y_minus] - [Phi_plus; Phi_minus] * [B_eq; tau_sf])^2;
s_squared = V_theta / (M - p);

% Covariance matrix
Phi_combined = [Phi_plus; Phi_minus];
cov_theta = s_squared * inv(Phi_combined' * Phi_combined);

% Confidence intervals
confidence_level = 1.96; % For 95% confidence
B_eq_CI = confidence_level * sqrt(cov_theta(1,1));
tau_sf_CI = confidence_level * sqrt(cov_theta(2,2));

fprintf('------------------------------------------------\n');
fprintf('Confidence Intervals (95%%):\n');
fprintf('------------------------------------------------\n');
fprintf('B_eq   ∈ [%.4e, %.4e] [Nm/(rad/s)]\n', B_eq - B_eq_CI, B_eq + B_eq_CI);
fprintf('tau_sf ∈ [%.4e, %.4e] [Nm]\n', tau_sf - tau_sf_CI, tau_sf + tau_sf_CI);
fprintf('------------------------------------------------\n');

%% Create Figure
figure;
hold on; grid on;

% Plot Positive Data
plot(omega_m_plus, tau_m_plus, 'b.', 'DisplayName', 'Positive Speed');

% Plot Negative Data
plot(omega_m_minus, tau_m_minus, 'r.', 'DisplayName', 'Negative Speed');

% Range of x values (rad/s)
w1 = linspace(-750, 0, 1000); % Adjust the range as needed
w2 = linspace(0, 750, 1000); % Adjust the range as needed


line1 = B_eq_plus * w2 + (1/N) * tau_sf_plus;
line2 = B_eq_minus * w1 - (1/N) * tau_sf_minus;

% Best-fit lines
plot(w2, line1, 'b', 'LineWidth', 2, 'DisplayName', 'Line 1: B_{eq+}, \tau_{sf+}');
plot(w1, line2, 'r', 'LineWidth', 2, 'DisplayName', 'Line 2: B_{eq-}, \tau_{sf-}');

% Labels and Title
xlabel('Motor Speed (RPM)', 'FontSize', 12);
ylabel('Friction Torque (Nm)', 'FontSize', 12);
title('Motor Speed vs. Friction Torque with Best-Fit Lines', 'FontSize', 14);
legend('show', 'Location', 'best');

% Axis limits
xlim([-750, 750]);
ylim([-2e-3, 2e-3]);

hold off;