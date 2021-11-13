%% D_SETPOINT_LIMIT
% 
% Consider the following setpoint signal: xd(t) = Ad*sin(2*pi*f*t), where
% Ad is the amplitude of the sine wave and f is the frequency of the sine
% wave. This script calculates the maximum amplitude, Ad, that the shake
% table stage can track given the frequency of the sine wave, f, as well
% as the table position, velocity, and acceleration limits.
%
% Input parameters:
%   p_lim     maximum +/- position (mm)
%   v_lim     maximum velocity (mm/s)
%   a_lim     maximum acceleration (m/s^2)
%
% Output variables:
%   f           frequency vector (Hz)
%   x_t         position limit based on physical stage stroke limit (mm)
%   x_v         position limit based on maximum velocity (mm)
%   x_a         position limit based on maximum acceleration (mm)
%   x_min       minimum between x_t, x_v, x_a (mm). This is the effective
%               maximum setpoint amplitude.
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
%%
function [f, x_t, x_v, x_a, x_min] = d_setpoint_limit(p_lim,v_lim,a_lim)
%
% calculate the acceleration limit of the shake table  (mm/s^2)
a_lim_mm = a_lim * 1000;
% generate frequency vector
f = 0.1:0.1:10;
% get size
n = max(size(f));
%
% create vectors for position, velocity, and acceleration limits
for i = 1:n
    x_t(i,1) =  p_lim;
    x_v(i,1) =  v_lim / ( 2 * pi * f(i) ); 
    x_a(i,1) =  a_lim_mm / (2 * pi * f(i) )^2;
    x_min(i,1) = min( [ x_t(i), x_a(i), x_v(i) ] );
end;

end




