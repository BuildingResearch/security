%% PLOT_RESPONSE
% Plots the response of variables in the Matlab workspace.
%
%% Load sample data from MAT files
% data saved: [t; u_amd; xc1; xc2; xf1_ddot; xf2_ddot; xf1; xf2; xd; x; Vm_tbl]
load('sample_data_stII_2xamd01.mat');
%
%% Setup variables
% Load saved response in the MAT file above.
t = data(1,:);              % time (s)
u1 = data(2,:);              % cart 1 voltage
u2 = data(3,:);              % cart 2 voltage
xc1 = data(4,:);             % measured cart 1 position
xc2 = data(5,:);             % measured cart 2 position
xf1 = data(6,:);            % floor 1 estimated deflection
xf2 = data(7,:);            % floor 2 estimated deflection
xc1_dot = data(8,:);         % cart 1 velocity
xc2_dot = data(9,:);         % cart 2 velocity
xf1_ddot = data(10,:);       % floor 1 acceleration 
xf2_ddot = data(11,:);       % floor 2 acceleration (relative to floor 1)
amd_mode = data(12,:);       % amd mode = active or passive
u_tbl = data(13,:);         % shake table voltage
xd = data(14,:);         % shake table desired position
x = data(15,:);          % shake table measured position
%
% Final plotting time (s)
tf = max(t);
%
%% Plot response
% cart 1 position response
figure(1)
subplot(2,1,1)
plot(t,1e3*xc1);
ylabel('x_{c1} (mm)');
% input voltage
subplot(2,1,2);
plot(t,u1);
ylabel('V_{m1} (V)');
xlabel('time (s)');

% cart 2 position response
figure(2)
subplot(2,1,1)
plot(t,1e3*xc2);
ylabel('x_{c2} (mm)');
% input voltage
subplot(2,1,2);
plot(t,u2);
ylabel('V_{m2} (V)');
xlabel('time (s)');


% floor 1 deflection and acceleration
figure(3)
subplot(3,1,1);
plot(t,1e3*xf1);
ylabel('x_{f1} (mm)');
subplot(3,1,2);
plot(t,xf1_ddot);
ylabel('x_{f1}\_ddot (m/s^2)');
subplot(3,1,3);
plot(t,amd_mode);
ylabel('amd mode');
xlabel('time (s)');
% 
% floor 2 deflection and acceleration
figure(4)
subplot(3,1,1);
plot(t,1e3*xf2);
ylabel('x_{f2} (mm)');
subplot(3,1,2);
plot(t,xf2_ddot);
ylabel('x_{f2}\_ddot (m/s^2)');
subplot(3,1,3);
plot(t,amd_mode);
ylabel('amd mode');
xlabel('time (s)');
% 
% table position
figure(5);
subplot(2,1,1);
plot(t,1e3*xd,t,1e3*x);
title('Table Position');
ylabel('x (mm)');
xlabel('time (s)');
legend('desired','measured');
subplot(2,1,2);
plot(t,u_tbl);
ylabel('Table Voltage (V)');
xlabel('time (s)');
% 
% Export to PNG files
print(1,'-dpng','-r300','stII_amd02_xc1.png');
print(2,'-dpng','-r300','stII_amd02_xc2.png');
print(3,'-dpng','-r300','stII_amd02_xf1.png');
print(4,'-dpng','-r300','stII_amd02_xf2.png');
print(5,'-dpng','-r300','stII_amd02_x.png');
% 