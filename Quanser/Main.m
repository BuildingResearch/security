% SETUP_STII_2xAMD1 for cyberattacks
%
% Shake Table II + Active Mass Damper Two-Floor, Two-Cart (2xAMD-1)
% Vibration Control Lab: Design of an observer-based state-feedback Controller
% 
% SETUP_STII_2xAMD1 sets the AMD-1 system model parameters accordingly to 
% the user-defined configuration and can also set the controller 
% parameters.
%
% Copyright (C) 2017 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
clear;
Tend = 20; % larger than Te
%% Earthquake
% name of data source file:
% El Centro
% input_filename = 'RSN180_IMPVALL.H_H-E05140.AT2';
% Cape Mendocino
% input_filename = 'RSN825_CAPEMEND_CPM000.AT2';
% Northridge
% input_filename = 'RSN1086_NORTHR_SYL090.AT2';
% Kobe
input_filename = 'RSN1105_KOBE_HIK000.AT2';

% Maximum scaled position (cm). NOTE: Set below 7.6 cm stroke limit.
x_max = 1.0;
make_quake
if Tend > Te
    Tstep = qc_get_step_size;
    Padd = length(Tc(end)+Tstep:Tstep:Tend);
    Tc = [Tc' Tc(end)+Tstep:Tstep:Tend]';
    xd = [xd; zeros(Padd,1)];
else
    Tend = Te;
end
%% USER-DEFINED 2xAMD-1 CONFIGURATION
% Type of motorized cart: set to 'IP01', 'IP02'
CART_TYPE = 'IP02';
% Type of Cart Load: set to 'NO_WEIGHT', 'ONE_WEIGHT', 'TWO_WEIGHT'
CART_LOAD_TYPE = 'TWO_WEIGHT';
% Turn on or off the safety watchdog on the cart position: set it to 1 , or 0 
XC_LIM_ENABLE = 1;       % safety watchdog turned ON
%XC_LIM_ENABLE = 0;      % safety watchdog turned OFF
% Safety Limits on the cart displacement (m)
XC_MAX = 0.08;            % cart displacement maximum safety position
XC_MIN = - XC_MAX;        % cart displacement minimum safety position
% Amplifier Gain used: set according to Gain Switch on VoltPAQ
K_AMD_AMP = 1;
% Amplifier Type: set to 'VoltPAQ'
AMP_TYPE = 'VoltPAQ';
%
%% USER-DEFINED ST II CONFIGURATION
% Enter revision of Shake Table II: REV1, REV2, REV3, or REV4.
% CAUTION: Only change this if you know for sure what revision the shake
% table being used is!
STII_REV = 'REV4';
% 
%% USER-DEFINED CONTROLLER DESIGN
% Type of Controller: set it to 'AUTO', 'MANUAL'  
CONTROLLER_TYPE = 'AUTO';    % PV + observer + state-feedback controller design: automatic mode
%CONTROLLER_TYPE = 'MANUAL';    % controller design: manual mode
% Design a Proportional-Velocity (PV) controller with following specs:
PO = 10;         % spec #1: maximum of percent overshoot
tp = 0.15;     % spec #2: time of first peak (s)
% 
% State-feedback controller design: calculate K using LQR
% X = [xc1 xc2 xf1 xf2 xc1_dot xc2_dot xf1_dot xf2_dot]
Q = diag([50 50 1 1 1 1 250 250]);
R = eye(2,2);
R(2,2)=2;

% desired full-order observer poles
OP = [ -55, -60, -65, -70, -75, -80, -85, -90 ]+40; % previous value 35
% 
%% SHAKE TABLE II PARAMETERS
% shake table II parameters
[SWEEP_MAX, P_MAX, VEL_MAX, ACC_MAX, VMAX_AMP, IMAX_AMP, wd, zd, wf, zf, wa, za, kp, kd, Kf, K_AMP, K_CURR, K_ENC, K_S0, K_S1, K_S2, K_MS2G, K_M2IN, K_G2MS, dbnc_threshold, dbnc_rise_time, hil_timeout ] = d_shake_table_II ();
% 
%% 2xAMD-1 MODEL
% initial state variables (at t=0)
X0 = [ 0; 0; 0; 0; 0; 0; 0; 0 ];
% Cart Encoder Resolution
global K_EC;
% Accelerometer gain (m/s^2)
global K_ACC;
% global At Bt Ct Dt Ab Bb Cb Db
[ Rm, Jm, Kt, Km, Kg, Mc, r_mp, Beq, Mf1, Kf1, Mf2, Kf2, VMAX_AMP_AMD, IMAX_AMP_AMD  ] = config_2xamd1( CART_TYPE, CART_LOAD_TYPE, AMP_TYPE );
Eff_m = 1;
Eff_g = 1;
% Design PV position control for carts (to keep them at a fixed position)
[ Kp, Kv ] = d_amd_pv( Rm, Jm, Kt, Eff_m, Km, Kg, Eff_g, Mc, r_mp, Beq, PO, tp );
% set state-space of 2xAMD-1 system
Two_AMD1_ABCD_eqns;
%
%% DISPLAY and CONTROL DESIGN
if strcmp ( CONTROLLER_TYPE, 'AUTO' )
    % PV controller design: automatic mode
    % Automatically design a Proportional-Velocity (PV) controller as per the user-specifications above
    % Automatically design the full-order observer and calculate the state-feedback gain vector as per the desired pole locations
    [ Kp_amd, Kv_amd, Kamd, G ] = d_2xamd1( Rm, Jm, Kt, Km, Kg, Mc, r_mp, Beq, PO, tp, A, B, C, D, OP, Q, R, X0 );
    % Display the calculated gains
    disp( ' ' )
    disp( 'Calculated PV controller gains: ' )
    disp( [ 'Kp = ' num2str( Kp_amd ) ' V/m' ] )
    disp( [ 'Kv = ' num2str( Kv_amd ) ' V.s/m' ] )
    disp( ' ' )
    disp( 'Calculated full-order observer matrix: ' )
    G
    disp( 'Calculated state-feedback gain vector: ' )
    Kamd
elseif strcmp ( CONTROLLER_TYPE, 'MANUAL' )
    disp( ' ' )
    disp( 'STATUS: manual mode' ) 
    disp( 'The model parameters of your AMD-2 system have been set.' )
    disp( 'You can now design your control system.' )
    disp( ' ' )
else
    error( 'Error: Please set the type of controller that you wish to implement.' )
end