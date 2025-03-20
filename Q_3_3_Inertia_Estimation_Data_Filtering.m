%% Load Data
% Data from Simulink
load('acceleration.mat');

% Acceleration dataset (Acceleration, Speed, Torque, Time)
acceleration_data = acceleration.Data;
time_acceleration = acceleration.Time;

% Extract columns
acceleration_values = acceleration_data(:,1); % Acceleration
w_a = acceleration_data(:,2); % Speed
tau_a = acceleration_data(:,3); % Torque

% Define valid time ranges
valid_ranges = [0.4, 1.95; 2.4, 2.95; 3.4, 3.95; 4.4, 4.95; 5.4, 5.95; ...
                6.4, 6.95; 7.4, 7.95; 8.4, 8.95; 9.4, 9.95; 10.4, 10.95; ...
                11.4, 11.95; 12.4, 12.95; 13.4, 13.95; 14.4, 14.95; 15.4, 15.95; ...
                16.4, 16.95; 17.4, 17.95; 18.4, 18.95; 19.4, 19.95]; % Valid time ranges

% Initialize filtered dataset
filtered_acceleration = [];

%% Process Acceleration Dataset
for range_idx = 1:size(valid_ranges,1) % Iterate through each valid range
    t_start = valid_ranges(range_idx,1);
    t_end = valid_ranges(range_idx,2);

    % Find indices within the valid time range
    valid_indices = (time_acceleration >= t_start) & (time_acceleration <= t_end);
    
    % Extract valid data
    valid_acceleration = acceleration_values(valid_indices);
    valid_w_a = w_a(valid_indices);
    valid_tau_a = tau_a(valid_indices);

    % Append unique valid data to the filtered dataset
    filtered_acceleration = [filtered_acceleration; valid_acceleration, valid_w_a, valid_tau_a];
end

%% Remove Duplicate Rows (If Any)
filtered_acceleration = unique(filtered_acceleration, 'rows'); 

%% Save Filtered Data
save('filtered_acceleration.mat', 'filtered_acceleration');
