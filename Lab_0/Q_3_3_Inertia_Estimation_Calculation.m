%% Load Filtered Acceleration Data
% load data to workspace
load('filtered_acceleration.mat');

% Extract acceleration, speed, and torque
acceleration_rpm_s = filtered_acceleration(:,1); % Acceleration in rpm/s
speed_rpm = filtered_acceleration(:,2); % Speed in rpm
tau_a = filtered_acceleration(:,3); % Torque

% Convert units to rad/s^2 and rad/s
acceleration_rad_s2 = acceleration_rpm_s * (2 * pi / 60); % Convert rpm/s to rad/s^2
speed_rad_s = speed_rpm * (2 * pi / 60); % Convert rpm to rad/s

% Define acceleration range for phase separation
accel_range = 2000 * (2 * pi / 60);

% Identify acceleration and deceleration phases
phase_boundaries = [min(acceleration_rad_s2):accel_range:max(acceleration_rad_s2)];
num_phases = length(phase_boundaries) - 1;
J_eq_n = zeros(num_phases, 1);

% Compute J_eq_n for each phase
for n = 1:num_phases
    % Find indices within the current phase range
    phase_indices = (acceleration_rad_s2 >= phase_boundaries(n)) & (acceleration_rad_s2 < phase_boundaries(n+1));
    
    if any(phase_indices)
        % Compute phase-wise \hat{\tau}_{i+} and \hat{\tau}_{i-}
        tau_plus_n = mean(tau_a(phase_indices & acceleration_rad_s2 > 0));
        tau_minus_n = mean(tau_a(phase_indices & acceleration_rad_s2 < 0));
        
        % Compute phase-wise a_{m+} and a_{m-}
        a_plus_n = mean(acceleration_rad_s2(phase_indices & acceleration_rad_s2 > 0));
        a_minus_n = mean(acceleration_rad_s2(phase_indices & acceleration_rad_s2 < 0));
        
        % Compute J_eq for the current phase
        if a_plus_n ~= a_minus_n
            J_eq_n(n) = (tau_plus_n - tau_minus_n) / (a_plus_n - a_minus_n);
        else
            J_eq_n(n) = NaN; 
        end
    else
        J_eq_n(n) = NaN; % No data in this phase
    end
end


J_eq_n = J_eq_n(~isnan(J_eq_n));
J_eq_calc = mean(J_eq_n);

% Display result
fprintf('------------------------------------------------\n');
disp('Equivalent Inertia Parameters');
fprintf('------------------------------------------------\n');
disp(['Equivalent Inertia (J_eq_calculated): ', num2str(J_eq_calc)]);
disp(['Equivalent Inertia (J_eq_given): ', num2str(J_eq)]);
fprintf('------------------------------------------------\n');