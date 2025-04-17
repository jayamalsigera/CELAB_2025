%%  Nominal tracking design


beta1 = real_part + 1i * imag_part;
beta2 = real_part - 1i * imag_part;
poles = [beta1, 
		beta2];		% complex-conjugate pair of poles

K = place(A, B, poles);	% State feedback gain

M = [A, B;
	C, D];				% Block Matrix for Reference Tracking (3x3 Matrix)

rhs = [0; 
	0; 
	1];					% Right-hand side
sol = M \ rhs;          % Solve the system (M * [Nx; Nu] = [0; 0; 1])

Nx = sol(1:2);          % State prefilter
Nu = sol(3);            % Input prefilter

Nr = Nu + K * Nx;		% Feedforward gain

%% Print Results
fprintf('===========================================================\n');
fprintf('---Nominal Tracking Design---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Desired Poles ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('λ1 = %.4f + %.4fj\n', real_part, imag_part); 
fprintf('λ2 = %.4f - %.4fj\n', real_part, imag_part);
fprintf('-----------------------------------------------------------\n');
fprintf('\n--- Augmented Matrix M = [A B; C 0] ---\n');
fprintf('-----------------------------------------------------------\n');
disp(M);
fprintf('-----------------------------------------------------------\n');
fprintf('--- State Feedback Gains ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('K = [%.6e; %.6e]\n', K(1), K(2));
fprintf('-----------------------------------------------------------\n');
fprintf('--- State Feedforward Gains ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Nx = [%.6e; %.6e]\n', Nx(1), Nx(2));
fprintf('Nu = %.6e\n', Nu);
fprintf('Nr = %.6e\n', Nr);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Tracking Gain ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('C*Nx = %.6e\n', C*Nx);
fprintf('===========================================================\n');
