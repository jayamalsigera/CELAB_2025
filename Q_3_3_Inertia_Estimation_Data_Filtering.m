remove = 1; 

% Extract data
% Positive dataset (Speed, Torque, Time)
wdot_m_plus = positive_wdot.Data(:,1); 
taudot_m_plus = positive_wdot.Data(:,2);
timedot_plus = positive_wdot.Time; 

% Negative dataset (Speed, Torque, Time)
wdot_m_minus = negative_wdot.Data(:,1); 
taudot_m_minus = negative_wdot.Data(:,2);
timedot_minus = negative_wdot.Time; 

% Remove the first second of data
filtered_tau_wdot_plus = [wdot_m_plus(timedot_plus > remove), taudot_m_plus(timedot_plus > remove)];
filtered_tau_wdot_minus = [wdot_m_minus(timedot_minus > remove), taudot_m_minus(timedot_minus > remove)];

% Save the filtered datasets
save('filtered_tau_wdot_plus.mat', 'filtered_tau_wdot_plus');
save('filtered_tau_wdot_minus.mat', 'filtered_tau_wdot_minus');