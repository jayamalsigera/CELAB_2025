%% Continuous-time PID controller with real derivative
s = tf('s');
C_s = K_P + K_I/s + K_D * (s / (T_L*s + 1));

%% Define z-transform variable
z = tf('z', T_sample);  

%% Discretization

% Using Tustin method
C_z_tustin = c2d(C_s, T_sample, 'tustin');

% Using Zero-Order Hold (ZOH) method
C_z_zoh = c2d(C_s, T_sample, 'zoh');

% Using Forward Euler method
s_forward = (z - 1) / T_sample;
C_z_forward = K_P + K_I / s_forward + K_D * (s_forward / (T_DL * s_forward + 1));

% Using Backward Euler method
s_backward = (z - 1) / (T_sample * z);
C_z_backward = K_P + K_I / s_backward + K_D * (s_backward / (T_DL * s_backward + 1));


%% Numerator and Denominator for Simulink
[numCd_tustin, denCd_tustin] = tfdata(C_z_tustin, 'v');
[numCd_zoh, denCd_zoh] = tfdata(C_z_zoh, 'v');
[numCd_forwardE, denCd_forwardE] = tfdata(C_z_forward, 'v');
[numCd_backwardE, denCd_backwardE] = tfdata(C_z_backward, 'v');

% Display results
disp('Tustin-discretized controller C(z):');
C_z_tustin

disp('ZOH-discretized controller C(z):');
C_z_zoh

disp('Forward Euler-discretized controller C(z):');
C_z_forward

disp('Backward Euler-discretized controller C(z):');
C_z_backward

% Plot pole-zero map for Tustin method
figure(1);
zplane(numCd_tustin, denCd_tustin);
title('Pole-Zero Map of C(z) - Tustin Method');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;

% Plot pole-zero map for ZOH method
figure(2);
zplane(numCd_zoh, denCd_zoh);
title('Pole-Zero Map of C(z) - Exact (ZOH) Method');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;

% Plot pole-zero map for Forward Euler method
figure(3);
zplane(numCd_forwardE, denCd_forwardE);
title('Pole-Zero Map of C(z) - Forward Euler Method');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;

% Plot pole-zero map for Backward Euler method
figure(4);
zplane(numCd_backwardE, denCd_backwardE);
title('Pole-Zero Map of C(z) - Backward Euler Method');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;


