%% Step 2: Reduced-Order Observer Design (2.2.2)
% Extract submatrices
Phi11 = Phi(1,1); Phi12 = Phi(1,2);
Phi21 = Phi(2,1); Phi22 = Phi(2,2);
Gamma1 = Gamma(1); Gamma2 = Gamma(2);

% Place observer eigenvalue
z_obs = 0.2;  % Fast pole
L = (Phi22 - z_obs) / Phi12;

% Observer matrices
Phi_o = Phi22 - L * Phi12;
Gamma_o = [Gamma2 - L*Gamma1, (Phi22 - L*Phi12)*L + Phi21 - L*Phi11];
H_o = [0 1];
J_o = [0 1; 0 L];

% Display
disp('Observer Gain L ='); disp(L);
disp('Phi_o ='); disp(Phi_o);
disp('Gamma_o ='); disp(Gamma_o);
disp('H_o ='); disp(H_o);
disp('J_o ='); disp(J_o);
