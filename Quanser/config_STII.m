%
%% CONFIG_STII
%
% Calculates the various system parameters associated with the Quanser
% Shake Table II system and the Quanser AMPAQ-PWM device. The user can
% define the revision # of the Shake Table II as well as the AMPAQ being
% used.
%
% Output parameters:
%   Rm          Motor resistance (ohms)
%   Kt          Current-torque constant (N.m/A)
%   Km          Back-emf motor constant (V/(rad/s))
%   Pb          Ball-screw pitch (m/rev)
%   Mp          Preload mass (kg)
%   Ml_max      Maximum total load mass (kg)
%   K_AMP       Amplifier gain (V/V)
%   Ms          Mass of the Shake Table II entire system (kg)
%   VMAX_AMP    Maximum output voltage of amplifier (V)
%   IMAX_AMP    Maximum output continuous current of amplifier (A)
%   P_MAX       Maximum stroke of STII stage (m)
%   K_ENC       Encoder sensitivity gain (m/count)
%   K_ACC       Accelerometer sensitivity gain (g/V)
%   K_CURR      Current sense gain (A/V)
%
% Copyright (C) 2015 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
% returns model parameters according to the USER-DEFINED HFLC system configuration
%
function [ Rm, Kt, Km, Pb, Mp, Ml_max, K_AMP, Ms, VMAX_AMP, IMAX_AMP, P_MAX ] = config_STII( )
% Calculate conversion factors
[ K_R2D, K_D2R, K_IN2M, K_M2IN, K_RDPS2RPM, K_RPM2RDPS, K_OZ2N, K_N2OZ, K_LBS2N, K_N2LBS, K_G2MS, K_MS2G ] = calc_conversion_constants ();
%
%% SHAKE TABLE II: Motor and stage paramters.
% Motor armature resistance (ohm)
Rm = 2.94;
% Back-emf motor constant (V/(rad/s))
Km = 23.4 / 1000 / K_RPM2RDPS;
% Motor Current-Torque Constant (N.m/A)
Kt = 0.360;
% Pitch of the ball-screw (m/rev)
Pb = 0.5 * K_IN2M;
% Preload mass = top stage + ball bearings (kg)
Mp = (5.18 + 4*0.64);
% Maximum load mass to meet specification of table (2.5 g at 20 Hz) (kg)
Ml_max = 33 * K_LBS2N / 9.81;
% Shake Table II Mass (kg)
Ms = 60 * K_LBS2N / 9.81;
% Maximum +/- travel of stage when starting at home position (m)
P_MAX = 3.0 * K_IN2M; % can move +/- 3 inches from HOME position.
% 
%% AMPLIFIER
% Maximum Output Current (A): AMPAQ-PWM can output peak current of 15 A
% Maximum rated current of STII motor is 15.6 A.
IMAX_AMP = 15.0;
% Maximum Output Voltage (V)
VMAX_AMP = Rm * IMAX_AMP;
% Amplifier gain (V/V) - duty cycle amplifier voltage gain 
% (note: found experimentally)
K_AMP = 10; %V/V set to 35 for higher payloads
% 
