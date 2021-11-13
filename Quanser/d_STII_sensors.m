%
%% D_STII_SENSORS
% This function calculates the encoder calibration gain given the encoder
% resolution, the direction, and the pitch if it is rotary.
%
% Input parameters:
%   Pb      ball-screw pitch (m)

% Output variables:
%   K_CURR  Current sense gain (A/V)
%   K_ENC   Encoder calibration gain (m/counts)
%   K_S0    Analog sensor gain connect to S0 (e.g. accelerometer, g/V)
%   K_S1    Analog sensor gain connect to S1 (e.g. accelerometer, g/V)
%   K_S2    Analog sensor gain connect to S2 (e.g. accelerometer, g/V)
%   K_S3    Analog sensor gain connect to S3 (e.g. accelerometer, g/V)
%   
%%
%
function [K_CURR, K_ENC, K_S0, K_S1, K_S2, K_S3] = d_STII_sensors(Pb)
%
% Current Sense (A/V)
K_CURR = 2.1;
%
% Encoder sensor gain (m/count).
K_ENC = Pb / 8192;
% note: pitch divided by encoder counts
% 
% ACCELEROMETERS
% Calibration gain for sensor connected to S1 input / Analog Input #0.
K_S0 = -1; % Note: default set to -1 g / V for table accelerometer
% Calibration gain for sensor connected to S2 input / Analog Input #1.
K_S1 = 1;
% Calibration gain for sensor connected to S3 input / Analog Input #2.
K_S2 = 1;
% Calibration gain for sensor connected to S4 input / Analog Input #3.
K_S3 = 1;
%