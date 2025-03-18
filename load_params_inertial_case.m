%% General Parameters and Conversion Gains

% Conversion gains
rpm2rads = 2*pi/60;          % [rpm] → [rad/s]
rads2rpm = 60/(2*pi);        % [rad/s] → [rpm]
rpm2degs = 360/60;           % [rpm] → [deg/s]
degs2rpm = 60/360;           % [deg/s] → [rpm]
deg2rad = pi/180;            % [deg] → [rad]
rad2deg = 180/pi;            % [rad] → [deg]
ozin2Nm = 0.706e-2;          % [oz*inch] → [N*m]

%% DC Motor Nominal Parameters

% Brushed DC Motor Faulhaber 2338S006S
mot.Ra = 2.6;                % Armature resistance [Ohm]
mot.Rs = 0.5;                % Shunt resistance [Ohm]
mot.L = 180e-6;              % Armature inductance [H]
mot.Kt = 1.088 * ozin2Nm;    % Torque constant [Nm/A]
mot.Ke = 0.804e-3 * rads2rpm;% Back-EMF constant [V/(rad/s)]
mot.J = 5.523e-5 * ozin2Nm;  % Rotor inertia [kg*m^2]
mot.B = 0.0;                 % Viscous friction coefficient (n.a.)
mot.eta = 0.69;              % Motor efficiency
mot.PN = 3.23/mot.eta;       % Nominal output power [W]
mot.UN = 6;                  % Nominal voltage [V]
mot.IN = mot.PN/mot.UN;      % Nominal current [A]
mot.tauN = mot.Kt*mot.IN;    % Nominal torque [Nm]
mot.taus = 2.42 * ozin2Nm;   % Stall torque [Nm]
mot.w0 = 7200 * rpm2rads;    % No-load speed [rad/s]

%% Gearbox Nominal Parameters

% Planetary gearbox Micromotor SA 23/1
gbox.N1 = 14;                % 1st reduction ratio (planetary gearbox)
gbox.eta1 = 0.80;            % Gearbox efficiency

% External transmission gears
gbox.N2 = 1;                 % 2nd reduction ratio (external transmission gears)
gbox.J72 = 1.4e-6;           % Inertia of a single external 72-tooth gear [kg*m^2]
gbox.eta2 = 1;               % External transmission efficiency (n.a.)

% Overall gearbox data
gbox.N = gbox.N1 * gbox.N2;  % Total reduction ratio
gbox.eta = gbox.eta1 * gbox.eta2; % Total efficiency
gbox.J = 3 * gbox.J72;       % Total inertia at gearbox output [kg*m^2]

%% Mechanical Load Nominal Parameters

% Inertia disc parameters
mld.JD = 3e-5;               % Load disc inertia [kg*m^2]
mld.BD = 0.0;                % Load viscous friction coefficient (n.a.)

% Overall mechanical load parameters
mld.J = mld.JD + gbox.J;     % Total inertia [kg*m^2]
mld.B = 2.5e-4;              % Total viscous friction coefficient (estimated)
mld.tausf = 1.0e-2;          % Total static friction [Nm] (estimated)

%% Voltage Driver Nominal Parameters

% Op-amp circuit parameters
drv.R1 = 7.5e3;              % Op-amp input resistor (DAC to non-inverting input) [Ohm]
drv.R2 = 1.6e3;              % Op-amp input resistor (non-inverting input to GND) [Ohm]
drv.R3 = 1.2e3;              % Op-amp feedback resistor (output to inverting input) [Ohm]
drv.R4 = 0.5e3;              % Op-amp feedback resistor (inverting input to GND) [Ohm]
drv.C1 = 100e-9;             % Op-amp input capacitor [F]
drv.outmax = 12;             % Op-amp max output voltage [V]

% Voltage driver DC-gain
drv.dcgain = drv.R2/(drv.R1 + drv.R2)*(1 + drv.R3/drv.R4);

% Voltage driver time constant
drv.Tc = drv.C1 * drv.R1 * drv.R2 / (drv.R1 + drv.R2);

%% Sensors Data

% Shunt resistor
sens.curr.Rs = 0.5;                                         % Shunt resistor [Ohm]

% Hewlett-Packard HEDS-5540#A06 optical encoder
sens.enc.ppr = 500*4;                                       % Pulses per rotation
sens.enc.pulse2deg = 360/sens.enc.ppr;                      % [pulses] → [deg]
sens.enc.pulse2rad = 2*pi/sens.enc.ppr;                     % [pulses] → [rad]
sens.enc.deg2pulse = sens.enc.ppr/360;                      % [deg] → [pulses]
sens.enc.rad2pulse = sens.enc.ppr/(2*pi);                   % [rad] → [pulses]

% Potentiometer 1 (Spectrol 138-0-0-103) installed on motor box
sens.pot1.range.R = 10e3;                                   % Ohmic value range [Ohm]
sens.pot1.range.V = 5;                                      % Voltage range [V]
sens.pot1.range.th_deg = 345;                               % Angle range [deg]
sens.pot1.range.th = sens.pot1.range.th_deg * deg2rad;      % Angle range [rad]
sens.pot1.deg2V = sens.pot1.range.V/sens.pot1.range.th_deg; % Sensitivity [V/deg]
sens.pot1.rad2V = sens.pot1.range.V/sens.pot1.range.th;     % Sensitivity [V/rad]
sens.pot1.V2deg = 1/sens.pot1.deg2V;                        % Conversion gain [V] → [deg]
sens.pot1.V2rad = 1/sens.pot1.rad2V;                        % Conversion gain [V] → [rad]

%% Data Acquisition Board (DAQ) Data

% NI PCI-6221 DAC data
daq.dac.bits = 16;                                  % Resolution (bits)
daq.dac.fs = 10;                                    % Full-scale voltage [V]
daq.dac.q = 2*daq.dac.fs/(2^daq.dac.bits - 1);      % Quantization step [V]

% NI PCI-6221 ADC data
daq.adc.bits = 16;                                  % Resolution (bits)
daq.adc.fs = 10;                                    % Full-scale voltage [V] (SLDRT Analog Input)
%%%%%%%%%%%%%%%%%
daq.adc.q = 2*daq.adc.fs/(2^daq.adc.bits - 1);      % Quantization step [V]
%%%%%%%%%%%%%%%%%%%
%% Equivalent Parameters

R_eq_model = mot.Ra + mot.Rs;                       % Equivalent Resistance (Shunt Resistance + Armature Resistance)
J_eq_model = mot.J + mld.J/gbox.N^2;                % Equivalent Moment of Inertia

q_en_1 = 360/(500*4);               % Quantization for all except MOTORE 8 and MOTORE 10
q_en_2 = 360/(1024*4);              % Quantization for MOTORE 8 and MOTORE 10
q_dac = 20/(2^16 - 1);              % Quantization for DAC