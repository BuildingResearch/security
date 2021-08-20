% ************************************************************************
% D_STII_LIMITS
%
% Computes the maximum velocity and force deliverable by the STII motor and
% the maximum acceleration given the total load mass.
%
% Input parameters:
%   Km          Back-emf constant (V/(rad/s))
%   Kt          Current-torque constant (N.m/A)
%   Mt          Total load mass (kg)
%   Pb          Ball-screw pitch (m/rev)
%   VMAX_UPM    Maximum output voltage of amplifier (V)
%   IMAX_UPM    Maximum output continuous current of amplifier (A)
%
% Output parameters:
%   VEL_MAX     Maximum linear velocity of stage (m/s)
%   F_MAX       Maximum linear force of stage (m/s^2)
%   ACC_MAX     Maximum linear acceleration of stage given load mass (m/s^2)
%   G_MAX       Maximum linear acceleration of stage given load mass (g)
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
% ************************************************************************
%
function [ VEL_MAX, F_MAX, ACC_MAX, G_MAX  ] = d_STII_limits( Km, Kt, Mt, Pb, IMAX_UPM, VMAX_UPM )
%
% Maximum angular velocity of motor (rad/s)
W_MAX = VMAX_UPM / Km; % 329 rad/s: less than rated max motor speed of 838 rad/s
% Maximum Linear Velocity (m/s)
VEL_MAX = W_MAX * Pb / ( 2 * pi );
% Maximum force (N)
F_MAX = Kt * IMAX_UPM / Pb;
% Maximum Acceleration (m/s^2)
ACC_MAX = F_MAX / Mt;
% Maximum Acceleration (g)
G_MAX = ACC_MAX / 9.81;