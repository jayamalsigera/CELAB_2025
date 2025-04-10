load_params_inertial_case;
SS_Calculations;


%% Q_2_2_SSC_Robust_Tracking_with_Extended_State_Estimator (FOR SINUSIOID)

Tr = 0.5;								% Sinusoid reference signal period [s]
omega_0 = 2 * pi / Tr;					% Angular frequency (rad/s)
alpha_z = [1 0 omega_0^2 0];			% Polynomial for: s^3 + ω₀² s

m = length(alpha_z) - 1;				% Internal model order (m = 3)
n = size(A,1);							% Plant order

%Az Calculation (Equation 67: Lab 1 Handout)
%Top-left block
As_top_left_top = [zeros(m-1,1), eye(m-1)];				% First m-1 rows of companion form
As_top_left_bottom = -fliplr(alpha_z(2:m+1));			% Last row = -α coefficients
As_top_left = [As_top_left_top; As_top_left_bottom];	% Full top-left block
%Top-right block
As_top_right = [zeros(m, n)];				% zeros
%Bottom blocks
As_bottom_left = [B, zeros(n, m-1)];				% Plant not affected by error dynamics
As_bottom_right = A;						% Original plant dynamics
%Combine
As = [As_top_left, As_top_right;			% Top half	
         As_bottom_left, As_bottom_right];	% Bottom half

As_num_rows = size(As, 1);
disp(['Number of rows in As: ', num2str(As_num_rows)]);

Bs = [zeros(m,1);
       B];

Bs_num_rows = size(Bs, 1);
disp(['Number of rows in Bs: ', num2str(Bs_num_rows)]);

Cs = [zeros(1, m), C];

C_p = [1, 0, 0];

lambda_e1 =  2 * wn * exp(1j * (-pi + 0.1*pi/3));
lambda_e2 = conj(lambda_e1);

lambda_e3 = 2* wn * exp(1j * (-pi + 0.01*pi/6));
lambda_e4 = conj(lambda_e3);

lambda_e5 = -2 * wn;

poles_ee = [lambda_e1, lambda_e2, lambda_e3, lambda_e4, lambda_e5];

Ls = place(As', Cs', poles_ee)';

beta1e = real_part + 1i * imag_part;
beta2e = conj(beta1e);
poles_ce = [beta1e, 
		beta2e];		% complex-conjugate pair of poles

Kce = place(A, B, poles_ce);

fprintf('===========================================================\n');
fprintf('--- Extended Estimator Model (Sinusoid) ---\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
fprintf('--- Extended Estimator System Matrices ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('As =\n'); 
disp(As);
fprintf('Bs =\n'); 
disp(Bs);
fprintf('Cs =\n'); 
disp(Cs);
fprintf('-----------------------------------------------------------\n');
fprintf('--- Extended Estimator Pole Placement ---\n');
fprintf('-----------------------------------------------------------\n');
for i = 1:length(poles_ee)
    fprintf('λ_c,%d = %.4f %+.4fj\n', i, real(poles_ee(i)), imag(poles_ee(i)));
end
fprintf('-----------------------------------------------------------\n');
fprintf('--- Extended Estimator Pole PlacementObserver Gain Matrix ---\n');
fprintf('-----------------------------------------------------------\n');
fprintf('Ls =\n');
disp(Ls);