%% Given accelerations
a_m_plus = 5; 
a_m_minus = -5;

%% Calculatiion

J_eq = (mean(filtered_tau_wdot_plus(:,2)) - mean(filtered_tau_wdot_minus(:,2))) / (a_m_plus - a_m_minus);


%% Display Results
fprintf('-----------------------------------\n');
%% Print Best-Fit Line Parameters
fprintf('J_eq = %.4e [Nm]\n', J_eq);
fprintf('-----------------------------------\n');