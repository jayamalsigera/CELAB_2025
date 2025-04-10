load_params_inertial_case;
SS_Calculations;

%% Robust tracking design with integral action

A_e = [0, C; zeros(size(A,1),1), A];     % 3x3 matrix
B_e = [0; B];							% 3x1 input matrix
C_e = [0, C];							% 1x3 output matrix
D_e = 0;  

% Pole placement (3 poles for 3x3 system)

beta1_e = 2.0*real_part + 2.5i * imag_part;
beta2_e = conj(beta1_e); 
beta3_e = 8*real_part; 

poles_e = [beta1_e, beta2_e, beta3_e];

% Pole placement for augmented system
Ke_e = place(A_e, B_e, poles_e);			% Design gain for augmented system

% Extract gains
KI_e = Ke_e(1);								% Integral gain
K_e = Ke_e(2:end);

% Print results
fprintf('===========================================================\n');
fprintf('---  Robust tracking design with integral action ---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Desired Poles ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('β1 = %.4f %+.4fj\n', real(beta1_e), imag(beta1_e));
fprintf('β2 = %.4f %+.4fj\n', real(beta2_e), imag(beta2_e));
fprintf('β3 = %.4f\n', beta3_e);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Selected Pole Placement ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Ke =');
disp(Ke_e);
fprintf('KI = %.6e\n', KI_e);
fprintf('Nx = [%.6e; %.6e]\n', Nx(1), Nx(2));
fprintf('Nu = %.6e\n', Nu);
fprintf('Nr = %.6e\n', Nr);
fprintf('-----------------------------------------------------------\n');
fprintf('===========================================================\n');

