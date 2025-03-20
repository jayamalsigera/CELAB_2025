%% Load Data

load('positive_speed.mat');
load('negative_speed.mat');

% Positive dataset (Speed, Torque, Time)
w_m_plus = positive_speed.Data(:,1); 
tau_m_plus = positive_speed.Data(:,2);
time_plus = positive_speed.Time; 

% Negative dataset (Speed, Torque, Time)
w_m_minus = negative_speed.Data(:,1); 
tau_m_minus = negative_speed.Data(:,2);
time_minus = negative_speed.Time; 

% Define step intervals
T_step = 5; % Periodic step time
valid_ranges = [0.5, 4.8; 5.2, 9.8; 10.2, 14.8; 15.2, 19.8; 20.2, 24.8; ...
                25.2, 29.8; 30.2, 34.8; 35.2, 39.8; 40.2, 44.8]; % Valid time ranges

% Initialize filtered datasets
filtered_positive = [];
filtered_negative = [];

%% Process Positive Dataset
for range_idx = 1:size(valid_ranges,1) % Iterate through each valid range
    t_start = valid_ranges(range_idx,1);
    t_end = valid_ranges(range_idx,2);

    % Find indices within the valid time range
    valid_indices = (time_plus >= t_start) & (time_plus <= t_end);
    
    % Extract valid data
    valid_tau_m = tau_m_plus(valid_indices);
    valid_w_m = w_m_plus(valid_indices);

    % Append unique valid data to the filtered dataset
    filtered_positive = [filtered_positive; valid_tau_m, valid_w_m];
end

%% Process Negative Dataset
for range_idx = 1:size(valid_ranges,1) % Iterate through each valid range
    t_start = valid_ranges(range_idx,1);
    t_end = valid_ranges(range_idx,2);

    % Find indices within the valid time range
    valid_indices = (time_minus >= t_start) & (time_minus <= t_end);
    
    % Extract valid data
    valid_tau_m = tau_m_minus(valid_indices);
    valid_w_m = w_m_minus(valid_indices);

    % Append unique valid data to the filtered dataset
    filtered_negative = [filtered_negative; valid_tau_m, valid_w_m];
end

%% Remove Duplicate Rows (If Any)
filtered_positive = unique(filtered_positive, 'rows'); 
filtered_negative = unique(filtered_negative, 'rows'); 

%% Save Filtered Data
save('filtered_positive.mat', 'filtered_positive');
save('filtered_negative.mat', 'filtered_negative');

