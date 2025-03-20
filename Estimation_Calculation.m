% Load Variable Parameters
load_params_inertial_case;

% Initial Calculations
PID_calculations;

%Friction Estimation
Q_3_2_Friction_Estimation_Data_Filtering
Q_3_2_Friction_Estimation_Calculation

% Inertia Estimation
Q_3_3_Inertia_Estimation_Data_Filtering;
Q_3_3_Inertia_Estimation_Calculation;