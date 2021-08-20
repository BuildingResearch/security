%%
% This script computes the open-loop model gain, Kf. The PD controller is
% then designed. Thus, using the model parameter Kf the proportional
% control gain, kp, and the derivative control gain, kd, are computed.
%
% Input parameters:
%   Kt      Current-torque constant (N.m/A) OR current-force constant (N/A)
%   Mt      Total load mass (kg)
%   Pb      Ball-screw pitch (m/rev)
%   f0      Desired closed-loop natural frequency (Hz)
%   zeta    Damping ratio
%
% Output parameters:
%   Kf      Open-loop model gain
%   kp      proportional control gain
%   kd      derivative control gain
%
% Copyright (C) 2017 Quanser Consulting Inc.
% Quanser Consulting Inc.
% 
%%
function [Kf, kp, kd] = d_STII_pd(Kt, Km, Mt, Pb, Rm, tp, PO)
%
%% Model
% Model gain for X(s) = 1/Kf/s^2 * Im(s)
Kf = Mt * Pb / Kt;
%
%% Damping Ratio and Natural Frequency
% i) spec #1: Percent Overshoot (PO)
if ( PO > 0 )
    % using the *Hint provided in the lab, zeta_min is given by:
    zeta_min = abs( log( PO / 100 ) ) / sqrt( pi^2 + log( PO / 100)^2 );
    zeta = zeta_min;
else
    error( 'Error: Set Percentage Overshoot.' )
end
% ii) spec #2: Peak Time, tp (s)
wn = pi / ( tp * sqrt( 1 - zeta^2 ) );
%
%% PD Gains
% Proportional gain (V/m)
% kp = wn^2 * Kf * Rm;
% Proportional control gain (A/m)
kp = wn^2 * Kf;
% Derivative gain (V-s/m)
% kd = - 2 * ( - zeta*Kf*Pb*Rm*wn + pi*Km) / Pb;
% Derivative control gain (A-s/m)
kd = 2*zeta*Kf*wn;
% 