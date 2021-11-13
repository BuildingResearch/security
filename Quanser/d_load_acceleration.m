%% D_LOAD_ACCELERATION
%
% This script computes the acceleration of the load given the maximum
% amplitude of the command position for a certain frequency.
%
% Input parameters:
%   f           Frequency vector (Hz)
%   Ad_max      Maximum command amplitude considering the maximum stroke,
%               velocity, and acceleration of the Shake Table II (mm).
%
% Output parameters:
%   g_max_f     Maximum acceleration vector at various frequencies (g).
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
%%
function [g_max_f] = d_load_acceleration(f,Ad_max)
% get size
n = max(size(f));
% Acceleration at maximum position command amplitudes (m/s^2)
for i = 1:n
    a_max_f(i) =  Ad_max(i) * ( 2 * pi * f(i) )^2 / 1000;
end;
%
% Convert accelerations (g)
g_max_f = a_max_f / 9.81;
%
end






