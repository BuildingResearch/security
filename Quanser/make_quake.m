%% MAKE_QUAKE
%
% Use this file to simulate an earthquake on the shake table system when
% the recorded data is in gravitational acceleration units (g). The q_scale
% algorithm is used to compute the desired position such that the measured
% accelerations yielded on the STII are equivalent to the recorded values.
%
% Copyright (C) 2012 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
%% INPUT
% name of data source file:
% El Centro
% input_filename = 'RSN180_IMPVALL.H_H-E05140.AT2';
% Cape Mendocino
% input_filename = 'RSN825_CAPEMEND_CPM000.AT2';
% Northridge
% input_filename = 'RSN1086_NORTHR_SYL090.AT2';
% Kobe
% input_filename = 'RSN1105_KOBE_HIK000.AT2';

% Maximum scaled position (cm). NOTE: Set below 7.6 cm stroke limit.
% x_max = 0.5;
%
%% Generate earthquake
% load recorded acceleration earthquake data in workspace (g) and sample
% time (s)
[dt,acc_data] = init_earthquake_data(input_filename);

% construct trajectory: 
% tq = recorded earthquake time (s)
% a = record earthquake acceleration (g)
[tq,a] = construct_quake_trajectory(acc_data,dt);
%
% Compute desired position of STII to achieve actual quake accelerations.
%   Tc =  Scaled time array (s)
%   Xc =  Scaled position array - use for table position command (cm)
%   Ac =  Scaled acceleration array using d2 Xc / dt2 (g)
%   Tu =  Earthquake time array (s)
%   Xu =  Computed earthquake displacement (not scaled) (cm)
%   Au =  Earthquake acceleration array (g), scaled to Tu (g)
[Tc, Xc, Ac, Tu, Xu, Au] = q_scale(tq, a, x_max);
%
% Duration (s)
Te = max(Tc);
% 
%% Plot
figure(2)
% Original recorded earthquake acceleration
subplot(2,2,1);
plot(Tu,Au);
title('Earthquake Acceleration (g)');
axis([0 max(Tu) 1.5*min(Au) 1.5*max(Au)]);
%
% Table acceleration from scaling (same as recorded)
subplot(2,2,2)
plot(Tc,Ac,'r-');
title('Table Acceleration (g)');
axis([0 max(Tc)  1.5*min(Ac) 1.5*max(Ac)]);
% 
% Original recorded earthquake position
subplot(2,2,3);
plot(Tu,Xu,'r-');
xlabel('time (s)');
title('Earthquake Position (cm)');
axis([0 max(Tu) 1.5*min(Xu) 1.5*max(Xu)]);
% 
% Scaled/command table position
subplot(2,2,4);
plot(Tc,Xc,'r-');
xlabel('time (s)');
title('Table Position (cm)');
axis([0 max(Tc) 1.5*min(Xc) 1.5*max(Xc)]);

%% Convert desired scaled position (m)
xd = Xc / 100;