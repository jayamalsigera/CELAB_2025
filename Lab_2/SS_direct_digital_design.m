load_params_inertial_case;
PID_calculations;
SS_Calculations;
SS_nominal_tracking;
discrete_time_basics;

%% === Discretize Using Exact Method (ZOH) ===
sys_c = ss(A, B, C, D);
sys_d_Ddd = c2d(sys_c, T_sample, 'zoh');

Phi_Ddd   = sys_d_Ddd.A;
Gamma_Ddd = sys_d_Ddd.B;
H_Ddd     = sys_d_Ddd.C;
J_Ddd     = sys_d_Ddd.D;

desired_poles_Ddd = [beta1 beta2];

L_Ddd = place(Phi_Ddd, Gamma_Ddd, desired_poles_Ddd);

%% === Display Results ===
fprintf('===========================================================\n');
fprintf('--- Direct Digital Design---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Direct Digital Design Matrices ---\n');
fprintf('-----------------------------------------------------------\n');
disp('Phi_Ddd ='); disp(Phi_Ddd);
disp('Gamma_Ddd ='); disp(Gamma_Ddd);
disp('H_Ddd ='); disp(H_Ddd);
disp('J_Ddd ='); disp(J_Ddd);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Feedback Gain ---\n');
fprintf('-----------------------------------------------------------\n');
disp('L_Ddd ='); disp(L_Ddd);
fprintf('-----------------------------------------------------------\n');

M_Ddd = [Phi_Ddd - eye(size(Phi_Ddd)), Gamma_Ddd;
	H_Ddd, 0];				% Block Matrix for Reference Tracking (3x3 Matrix)

rhs_Ddd = [0; 
	0; 
	1];					% Right-hand side
sol_Ddd = M_Ddd \ rhs_Ddd;          % Solve the system (M * [Nx; Nu] = [0; 0; 1])

Nx_Ddd = sol_Ddd(1:2);          % State prefilter
Nu_Ddd = sol_Ddd(3);            % Input prefilter

Nr_Ddd = Nu_Ddd + K * Nx_Ddd;		% Feedforward gain

fprintf('-----------------------------------------------------------\n');
fprintf('--- State Feedforward Gains ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Nx_Ddd = [%.6e; %.6e]\n', Nx_Ddd(1), Nx_Ddd(2));
fprintf('Nu_Ddd = %.6e\n', Nu_Ddd);
fprintf('Nr_Ddd = %.6e\n', Nr_Ddd);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Tracking Gain ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('H_Ddd*Nx_Ddd = %.6e\n', H_Ddd*Nx_Ddd);
fprintf('===========================================================\n');