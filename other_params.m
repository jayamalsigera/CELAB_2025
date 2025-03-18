R_s_model = 0;                      % Shunt Resistance
R_a_model = mot.R;                  % Armature Resistance
N_model = 14;                       % Number of Poles
t_sf_model = 1.0e-2;                % Static Friction Coefficient
B_eq_model = 2.0e-6;                % Equivalent Friction Coefficient

R_eq_model = R_a_model + R_s_model; % Equivalent Resistance (Shunt Resistance + Armature Resistance)
J_eq_model = mot.J + mld.J/N_model^2;     % Equivalent Moment of Inertia

q_en_1 = 360/(500*4);               % Quantization for all except MOTORE 8 and MOTORE 10
q_en_2 = 360/(1024*4);              % Quantization for MOTORE 8 and MOTORE 10
q_dac = 20/(2^16 - 1);              % Quantization for DAC

