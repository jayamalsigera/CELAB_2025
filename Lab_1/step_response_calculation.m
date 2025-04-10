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
