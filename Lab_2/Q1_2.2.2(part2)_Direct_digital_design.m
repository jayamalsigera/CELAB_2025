%% Step 3: Design Controller Gains (2.2.2)
% Desired closed-loop poles
z1 = 0.6;
z2 = 0.65;

% State feedback gain K
K = place(Phi, Gamma, [z1, z2]);

% Solve for feedforward gains Nx, Nu
M = [Phi - eye(2), Gamma; H, 0];
sol = M \ [zeros(2,1); 1];
Nx = sol(1:2);
Nu = sol(3);
Nr = Nu + K*Nx;

% Display
disp('State Feedback Gain K ='); disp(K);
disp('Feedforward Gains:');
disp('Nx ='); disp(Nx);
disp('Nu ='); disp(Nu);
disp('Nr ='); disp(Nr);
