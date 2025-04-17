%% Controller State Space Matrix Dimensions

nx = size(A, 1);
mx = size(B, 2);
px = size(C, 1);

%% State Transformation Matrix

V_ro = [-C(2), C(1)];
T_ro_inv = [C; V_ro];
T_ro = inv(T_ro_inv);

%% Reduced-Order Transformed System

A_ro_bar = inv(T_ro) * A * T_ro;
B_ro_bar = T_ro * B;

A11_bar = A_ro_bar(1:px, 1:px);
A12_bar = A_ro_bar(1:px, px+1:nx);
A21_bar = A_ro_bar(px+1:nx, 1:px);
A22_bar = A_ro_bar(px+1:nx, px+1:nx);

B1_bar = B_ro_bar(1:px, :);
B2_bar = B_ro_bar(px+1:nx, :);

%% Observer Gain Placement

lambda_ro = -5 * w_n;

L_ro = place(A22_bar, A12_bar, lambda_ro);

T_ro = eye(nx);

%% Observer Matrices Derivation
% === A_o
A_o = A22_bar - L_ro * A12_bar;

% === B_o
B_o1 = B2_bar - L_ro * B1_bar;
B_o2 = A_o * L_ro + A21_bar - L_ro * A11_bar;
B_o = [B_o1, B_o2];

% === C_o
C_o1 = eye(nx - px);
C_o2 = zeros(px, nx - px);
C_o = T_ro * [C_o2; C_o1];

% === D_o
D1 = [zeros(px, mx), eye(px)];
D2 = [zeros(nx - px, mx), L_ro];
D_o = T_ro * [D1; D2];

%% Observer Matrices Derivation from given expressions

% A_o_calc
A_o_calc = -(1/T_m) - L_ro;

% B_o_calc
B_o_calc1 = k_m / (gbox.N * T_m);
B_o_calc2 = A_o_calc * L_ro;
B_o_calc = [B_o_calc1, B_o_calc2];

% C_o_calc
C_o_calc = [0; 1];

% D_o_calc
D_o_calc = [0, 1; 0, L_ro];

%% Display All Observer Matrices
fprintf('===========================================================\n');
fprintf('Reduced Order Observer\n');
fprintf('===========================================================\n');
fprintf('-----------------------------------------------------------\n');
disp('A_o (transformed):'); disp(A_o);
disp('A_o (calculated):'); disp(A_o_calc);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('B_o (transformed):'); disp(B_o);
disp('B_o (calculated):'); disp(B_o_calc);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('C_o (transformed):'); disp(C_o);
disp('C_o (calculated):'); disp(C_o_calc);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('D_o (transformed):'); disp(D_o);
disp('D_o (calculated):'); disp(D_o_calc);
fprintf('-----------------------------------------------------------\n');

%% Discrete-time observer matrices

I = eye(size(A_o));
sys_ct = ss(A_o, B_o, C_o, D_o);

%% === Forward Euler ===
Phi_o_forwardE = I + A_o * T_sample;
Gamma_o_forwardE = B_o * T_sample;
H_o_forwardE = C_o;
J_o_forwardE = D_o;

%% === Backward Euler ===
Phi_o_backwardE = inv(I - A_o * T_sample);
Gamma_o_backwardE = Phi_o_backwardE * B_o * T_sample;
H_o_backwardE = C_o * Phi_o_backwardE;
J_o_backwardE = D_o + C_o * Phi_o_backwardE * B_o * T_sample;

%% === Trapezoidal (Tustin) ===
Phi_o_tustin = inv(I - A_o * T_sample / 2) * (I + A_o * T_sample / 2);
Gamma_o_tustin = inv(I - A_o * T_sample / 2) * B_o * T_sample;
H_o_tustin = C_o * inv(I - A_o * T_sample / 2);
J_o_tustin = D_o + C_o * inv(I - A_o * T_sample / 2) * B_o * T_sample;

% === Discretize using Tustin (Bilinear / Trapezoidal) method
sys_d_tustin = c2d(sys_ct, T_sample, 'tustin');

% === Extract the matrices
Phi_o_tustin_ = sys_d_tustin.A;
Gamma_o_tustin_ = sys_d_tustin.B;
H_o_tustin_ = sys_d_tustin.C;
J_o_tustin_ = sys_d_tustin.D;

%% === Exact (ZOH) ===
sys_ct = ss(A_o, B_o, C_o, D_o);
sys_d_exact = c2d(sys_ct, T_sample, 'zoh');

Phi_o_exact = sys_d_exact.A;
Gamma_o_exact = sys_d_exact.B;
H_o_exact = sys_d_exact.C;
J_o_exact = sys_d_exact.D;
fprintf('-----------------------------------------------------------\n');
disp('FORWARD EULER');
fprintf('-----------------------------------------------------------\n');
disp('Phi_o_forwardE ='); disp(Phi_o_forwardE);
disp('Gamma_o_forwardE ='); disp(Gamma_o_forwardE);
disp('H_o_forwardE ='); disp(H_o_forwardE);
disp('J_o_forwardE ='); disp(J_o_forwardE);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('BACKWARD EULER');
fprintf('-----------------------------------------------------------\n');
disp('Phi_o_backwardE ='); disp(Phi_o_backwardE);
disp('Gamma_o_backwardE ='); disp(Gamma_o_backwardE);
disp('H_o_backwardE ='); disp(H_o_backwardE);
disp('J_o_backwardE ='); disp(J_o_backwardE);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('TUSTIN (MANUAL)');
fprintf('-----------------------------------------------------------\n');
disp('Phi_o_tustin ='); disp(Phi_o_tustin);
disp('Gamma_o_tustin ='); disp(Gamma_o_tustin);
disp('H_o_tustin ='); disp(H_o_tustin);
disp('J_o_tustin ='); disp(J_o_tustin);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('TUSTIN (c2d)');
fprintf('-----------------------------------------------------------\n');
disp('Phi_o_tustin_ ='); disp(Phi_o_tustin_);
disp('Gamma_o_tustin_ ='); disp(Gamma_o_tustin_);
disp('H_o_tustin_ ='); disp(H_o_tustin_);
disp('J_o_tustin_ ='); disp(J_o_tustin_);
fprintf('-----------------------------------------------------------\n');

fprintf('-----------------------------------------------------------\n');
disp('EXACT (ZOH)');
fprintf('-----------------------------------------------------------\n');
disp('Phi_o_exact ='); disp(Phi_o_exact);
disp('Gamma_o_exact ='); disp(Gamma_o_exact);
disp('H_o_exact ='); disp(H_o_exact);
disp('J_o_exact ='); disp(J_o_exact);
fprintf('-----------------------------------------------------------\n');