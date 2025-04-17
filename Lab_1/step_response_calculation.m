% Extract time and signal vectors
t = thl_meas.time;
y = thl_meas.signals.values;
r_final = 70;  % Step amplitude

% Calculate time-domain performance metrics
info = stepinfo(y, t, r_final);

% Display results
fprintf('Rise Time: %.4f s\n', info.RiseTime);
fprintf('Peak Time: %.4f s\n', info.PeakTime);
fprintf('Settling Time: %.4f s\n', info.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);
fprintf('Peak Value: %.4f s\n', info.Peak);

peak_time = info.PeakTime;
peak_value = info.Peak;
% Find index after peak time
after_peak_idx = find(t > peak_time);

% Undershoot: minimum after the peak
min_after_peak = min(y(after_peak_idx));
undershoot = max(0, (r_final - min_after_peak) / r_final * 100);  % In percentage

% Optional: also calculate settling times
settling_time_5 = info.SettlingTime;
settling_time_1 = stepinfo(y, t, r_final, 'SettlingTimeThreshold', 0.01).SettlingTime;

% Display
fprintf("Peak Value: %.4f\n", peak_value);
fprintf("Undershoot (after peak): %.2f %%\n", undershoot);
fprintf("5%% Settling Time: %.4f s\n", settling_time_5);
fprintf("1%% Settling Time: %.4f s\n", settling_time_1);

