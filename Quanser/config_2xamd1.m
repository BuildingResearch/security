% CONFIG_2xAMD1
%
% CONFIG_2xAMD1 accepts the user-defined configuration 
% of the Quanser's Two-Floor Active Mass Damper system with 2 carts. 
% CONFIG_2xAMD1 accordingly sets up and returns the 
% configuration-dependent model parameters of the Quanser 2xAMD-1 plant.
%
% 2xAMD-1 system nomenclature:
% Rm        Cart Motor Armature Resistance                      (Ohm)
% Jm        Cart Rotor Inertia                                  (kg.m^2)
% Kt        Cart Motor Torque Constant                          (N.m/A)
% Km        Cart Motor Back-EMF Constant                        (V.s/rad)
% Kg        Cart Planetary Gearbox Gear Ratio
% Mc        Total Mass of the Cart System (i.e. moving parts)   (kg)
% r_mp      Cart Motor Pinion Radius                            (m)
% Beq       Cart Equivalent Viscous Damping Coefficient 
%                       as seen at the Motor Pinion             (N.s/m)
% Mf1       First Floor Assembly Mass                           (kg)
% Kf1       First Floor Linear Stiffness Constant               (N/m)
% Mf2       Second Floor Assembly Mass                          (kg)
% Kf2       Second Floor Linear Stiffness Constant              (N/m)
% K_PC      if IP01: Cart Potentiometer Sensitivity             (m/V)
% K_EC      if IP02: Cart Encoder Resolution                    (m/count)
% K_ACC     Floor Accelerometer Sensitivity                     (m/s^2/V)
% VMAX_AMP  Amplifier Maximum Output Voltage                    (V)
% IMAX_AMP  Amplifier Maximum Output Current                    (A)
%
% Copyright (C) 2006 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
%
%% returns the model parameters accordingly to the USER-DEFINED AMD2 system configuration
function [ Rm, Jm, Kt, Km, Kg, Mc, r_mp, Beq, Mf1, Kf1, Mf2, Kf2, VMAX_AMP, IMAX_AMP  ] = config_2xamd1( CART_TYPE, CART_LOAD_TYPE, AMP_TYPE )
% Calculate useful conversion factors
calc_conversion_constants;
global K_IN2M
% Sets up the AMD cart model parameters accordingly to the user-defined configuration.
[ Rm, Jm, Kt, Eff_m, Km, Kg, Eff_g, Mc, r_mp, Beq, VMAX_AMP, IMAX_AMP  ] = setup_amd_cart_parameters( CART_TYPE, CART_LOAD_TYPE, AMP_TYPE );
% First Floor Height (m)
Hf1 = 19.75 * K_IN2M; % = 0.502
% Second (i.e. Top) Floor Height (m)
Hf2 = ( 21 + 19.75 ) * K_IN2M; % = 1.035
% Structure Total Mass (with accelerometers but with no rack and no cart) (kg)
Ms = 3.30;
% First Floor Mass (kg)
Mf1 = 1.16;
% Rack Mass (kg)
Mr = 0.7;
% First Floor w/ rack Total Mass (kg)
Mf1 = Mf1 + Mr;
% Top/Second Floor Mass (kg)
Mf2 = 1.16; %0.68;
% Top Floor/Roof (i.e. floor #2 + rack) Total Mass (kg)
Mf2 = Mr + Mf2; % = 1.38
% First Floor Linear Stiffness Constant (To The Ground) (N/m)
Kf1 = 195.6; % based on experimental test on AMD01 (old: 500 N/m)
% Second Floor Linear Stiffness Constant (To The First Floor) (N/m)
Kf2 = 195.6; % based on experimental test on AMD01 (old: 500 N/m)
% 
% Floor Accelerometer Sensitivity (m/s^2/V)
global K_ACC
% accelerometer calibration: 1.0 g/V
% positive floor acceleration is defined to the right when facing the AMD-2
K_ACC = 9.81;
% end of 'setup_amd2_parameters( )'


%% returns the model parameters accordingly to the USER-DEFINED AMD system configuration
function [ Rm, Jm, Kt, Eff_m, Km, Kg, Eff_g, Mc, r_mp, Beq, VMAX_AMP, IMAX_AMP ] = setup_amd_cart_parameters( CART_TYPE, CART_LOAD_TYPE, AMP_TYPE )
global K_IN2M K_D2R K_RDPS2RPM K_OZ2N 
% Motor Armature Resistance (Ohm)
Rm = 2.6;
% Motor Armature Inductance (H)
Lm = 180e-6;
% Motor Torque Constant (N.m/A)
Kt = 1.088 * K_OZ2N * K_IN2M; % = .00767
% Motor ElectroMechanical Efficiency [ = Tm * w / ( Vm * Im ) ]
Eff_m = 1;
% Motor Back-EMF Constant (V.s/rad)
Km = 0.804e-3 * K_RDPS2RPM; % = .00767
% Rotor Inertia (kg.m^2)
Jm = 5.523e-5 * K_OZ2N * K_IN2M; % = 3.9e-7
% Cart (IP01 or IP02) Mass, with 2 cable connectors (kg)
Mc = 0.39;
% Cart Weight Mass (kg)
Mw = 0.13;
% Planetary Gearbox (a.k.a. Internal) Gear Ratio
Kg = 3.71;
% Planetary Gearbox Efficiency
Eff_g = 1;
% Cart Motor Pinion number of teeth
N_mp = 24;
% Motor Pinion Radius (m)
r_mp = 0.5 / 2 * K_IN2M;  %  = 6.35e-3
% Cart Position Pinion number of teeth
N_pp = 56;
% Position Pinion Radius (m)
r_pp = 1.167 / 2 * K_IN2M; %  = 14.8e-3
% Rack Pitch (m/teeth)
Pr = 1e-2 / 6.01; % = 0.0017
% Cart Travel (m)
Tc = 0.19;
% Set the following calibration constants/parameters: K_EC, K_PC, Beq
if strcmp( CART_TYPE, 'IP01')
    % Cart Potentiometer Sensitivity (m/V)
    global K_PC
    % the cart position potentiometer goes from -5V to +5V in 10 turns
    K_PC = 2 * pi * r_pp * 10 / 10; % = 0.0931
elseif strcmp ( CART_TYPE, 'IP02')
    % Cart Encoder Resolution (m/count)
    global K_EC
    K_EC = Pr * N_pp / ( 4 * 1024 ); % = 22.7485 um/count
else 
    error( 'Error: Set the type of motorized cart.' )
end
% Equivalent Viscous Damping Coefficient as seen at the Motor Pinion (N.s/m)
Beq = 3;
% Cart Total Mass (kg)
if strcmp ( CART_LOAD_TYPE, 'ONE_WEIGHT' )
    Mc = Mc + Mw;
elseif strcmp ( CART_LOAD_TYPE, 'TWO_WEIGHT' )
    Mc = Mc + 2 * Mw;
else 
    error( 'Error: Set the AMD cart load configuration.' )
end
%
% Set the Amplifier Maximum Output Voltage (V) and Output Current (A)
% rm: for low values of K_AMP, VMAX_AMP is limited by VMAX_DAC
if  strcmp( AMP_TYPE, 'VoltPAQ' )
    VMAX_AMP = 24;
    IMAX_AMP = 4;
elseif  strcmp( AMP_TYPE, 'UPM_2405' )
    VMAX_AMP = 22;
    IMAX_AMP = 5;
elseif ( strcmp( AMP_TYPE, 'UPM_1503' ) | strcmp( AMP_TYPE, 'UPM_1503x2' ) ) 
    VMAX_AMP = 13;
    IMAX_AMP = 3;
else
    error( 'Error: Set the amplifier type.' )
end
% end of 'setup_amd_cart_parameters( )'
%
%% Calculate Useful Conversion Factors w.r.t. Units
function calc_conversion_constants ()
global K_D2R K_IN2M K_RDPS2RPM K_OZ2N
% from radians to degrees
K_R2D = 180 / pi;
% from degrees to radians
K_D2R = 1 / K_R2D;
% from Inch to Meter
K_IN2M = 0.0254;
% from Meter to Inch
K_M2IN = 1 / K_IN2M;
% from rad/s to RPM
K_RDPS2RPM = 60 / ( 2 * pi );
% from RPM to rad/s
K_RPM2RDPS = 1 / K_RDPS2RPM;
% from oz-force to N
K_OZ2N = 0.2780139;
% from N to oz-force
K_N2OZ = 1 / K_OZ2N;
% end of 'calc_conversion_constants( )'
