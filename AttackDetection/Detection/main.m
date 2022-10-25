%% Benchmark 20-story building structure with an ATMD system
clear all;clc;
close all;
addpath('data')
% States: X = [20 disp, 1 AMD disp, 20 vel, 1 AMD vel]
%% Parameters
% Earthquake intensity
EI = 1;
% Max. actuator force
MaxF = 2000e3; 
% AMD mass
miu = 0.02; % Mass ratio between 1% and 5%
Max_AMD_Disp=0.5;   % Max. stroke
kstop=4e6;          % Stiffness of stop force
cstop=5e7;          % Damping of stop force
% LQR gain
r_lqr = 10^-14;

% FDI attack
Max_AMD_Disp=0.5;
FDIon = 0;  % 0: off, 1:on
DoS_on = 0; % 0:off, 1:on
% FDI_amp = 1000e5; % miu = 0.02
% FDI_amp = 900e2; % miu  = 0.01
FDI_amp = 300e3;
FDI_fr = 1/3.5104;  % first nat. frequency in Hz

%% Structural Properties
% Mass(10^6) Kg
m(1)= 1.126; 
m(2:19)=1.1;
m(20)= 1.170;
m=m*1e6; %kg

% Interstory stiffness (10^3) KN/m;
k(1:5) = 862.07; 
k(6:11) = 554.17;
k(12:14) = 453.51;
k(15:17) = 291.23;
k(18:19) = 256.46;
k(20)=171.70;
k=k*1e3*1e3; % N/m

%% Characteristic matrices
% Mass Matrix
M = diag(m); 

% Stiffness Matrix
K=zeros(length(k)); 
for i=1:length(K)
    for j=1:length(K)
        if j==length(K) && i ==length(K)
            K(j,j)= k(end);
        elseif i == j
            K(i,i)=k(i)+k(i+1);  
        end
    end
end

for i=1:length(K)-1
    K(i,i+1)=-k(i+1);
end

for j=1:length(K)-1
    K(j+1,j)=-k(j+1);
end

% Damping matrix: Rayleigh using 5% in first two modes
[Phi,w2]=eig(K,M);      
[Wn,ord]=sort(sqrt(diag(w2)));
Tn=2*pi./Wn;
Phi=Phi(:,ord);

w1= Wn(1);
w2= Wn(2);

W=[1/w1 w1; 1/w2 w2];
zeta = [0.05;0.05];

syms aa0 aa1
[aa0,aa1]=solve(W*[aa0;aa1]==zeta);
aa0=sym2poly(aa0);
aa1=sym2poly(aa1);

Cc = aa0*M + aa1*K; % Damping Matrix
%% ATMD model
zet = 0.05;     % structure damping ratio

% Tuning using Sadek (1997)
gamma = 1/(miu+1)*(1-zet*sqrt(miu/(1+miu)));
zet_tmd = zet/(miu+1)+sqrt(miu/(1+miu));

Mef=(Phi'*M*ones(20,1)).^2./diag(Phi'*M*Phi);  % Modal effective mass

mf=miu*Mef(1);   % kg
wf=gamma*Wn(1);
kf=wf^2*mf;
cf=zet_tmd*2*wf*mf;

% Matrices
n=20;
M_tlcd=[M zeros(n,1);zeros(1,n) mf];
C_tlcd=[Cc [zeros(n-1,1);-cf];[zeros(1,n-1) -cf] cf];
K_tlcd=[K [zeros(n-1,1);-kf];[zeros(1,n-1) -kf] kf];

%% Uncontrolled State space model
iota=ones(20,1);
A_ref = [...
    zeros(n,n) eye(n);
    -K/M -Cc/M];
E_ref = [...
    zeros(n,1);
    -eye(n)\iota];
Dri=diag(ones(n,1));    % drift computation for C
Dri(2:n+1:end)=-ones(n-1,1);
% C_ref = [Dri zeros(n)];
C_ref = [...
    eye(n) zeros(n,n);
    zeros(n,n) eye(n);
    Dri zeros(n);
    -K/M -Cc/M];
DD_ref = [zeros(4*n,1)];
Sys_unc = ss(A_ref,E_ref,C_ref,DD_ref);
  
%% AMD state space model
n_amd=n+1;
A = [...
    zeros(n_amd) eye(n_amd);
    -K_tlcd/M_tlcd -C_tlcd/M_tlcd];

E = [...                    % for earthquake input (f)
    zeros(n_amd,1); 
    -eye(n)\iota;
    0];
B=[...                      % for control input (u)
    zeros(n_amd,1);
    M_tlcd\[zeros(n-1,1);-1;1]];       
Dri=diag(ones(n_amd,1));
Dri(2:n_amd+1:end)=-ones(n_amd-1,1);
Dri(n_amd,end-1)=0;
% C = [...
%     Dri zeros(n_amd)];
C = [...
    eye(n_amd) zeros(n_amd);
    zeros(n_amd) eye(n_amd);
    Dri zeros(n_amd);
    -K_tlcd/M_tlcd -C_tlcd/M_tlcd];
D = [zeros(4*n_amd,1+1)];
Sys_con = ss(A,[E B],C,D);
h = [5.49; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96;...
    3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96; 3.96]; % Floor heights
%% Controller (LQR example)
R = r_lqr;
Q = eye(2*n_amd);
K_lqr = lqr(A,B,Q,R);

%% Earthquake
load('Elcentro')
EQ.signals.values=Acc(1:4:end);
EQ.time=t(1:4:end);

tend=EQ.time(end);
TsEarthquake=EQ.time(2);
%% Attack Detection
nx = length(A);
nu = size(B);
nu = nu(1);
attack = ones(nu,1);
attack(10:12) = 0;
Ts = 0.01;
%% Simulation Attack detection - Detector 2 Measuring the earthquake

iState = 24;
FontSize = 16;
load('Detector2.mat')
EI = 1;
% Attack DoS
DoS_on = 1; % 0:off, 1:on
FDIon  = 0; % 0: off, 1:on
sim ('B20_ATMD_Earthquake')
CUSUM_Earthquake_DoS = CUSUM;
generatePlot(CUSUM_Earthquake_DoS, D_con, threshold, Alarms, h, t)

% No Attack 
DoS_on = 0; % 0:off, 1:on
FDIon  = 0; % 0: off, 1:on
sim ('B20_ATMD_Earthquake')
CUSUM_Earthquake_NoA = CUSUM;
figure
stairs(CUSUM_Earthquake_NoA.time, CUSUM_Earthquake_NoA.data(:,iState), 'LineWidth', 1.5), hold on
stairs(CUSUM_Earthquake_NoA.time, CUSUM_Earthquake_NoA.data(:,iState)*0 + threshold(iState), '--k', 'LineWidth', 1)
xlabel('Time [s]', 'Interpreter', 'latex'), ylabel('$S_{24}$', 'Interpreter','latex')
ylim([0 threshold(iState)*5])
set(gca,'fontsize',FontSize)
set(gcf,'position',[403   246   500   320])

ttd_Earthquake_NoA = timeToDetect( Alarms );

% FDI no earthquake
EI = 0;
DoS_on = 0; % 0:off, 1:on
FDIon  = 1; % 0: off, 1:on
sim ('B20_ATMD_Earthquake')
CUSUM_NoEarthquake_FDI = CUSUM;
generatePlot(CUSUM_NoEarthquake_FDI, D_con, threshold, Alarms, h, t)

%% Simulation Attack detection - Detector 1 Withouth measuring the earthquake
% Load CUSUM parameters for the first detector
load('Detector1.mat')
EI = 1;
% Attack DoS
DoS_on = 1; % 0:off, 1:on
FDIon  = 0; % 0: off, 1:on
sim ('B20_ATMD_NoEarthquake')
CUSUM_Earthquake_NoMeas_DoS = CUSUM;
generatePlot(CUSUM_Earthquake_NoMeas_DoS, D_con, threshold, Alarms, h, t)

% No Attack Earthquake
DoS_on = 0; % 0:off, 1:on
FDIon  = 0; % 0: off, 1:on
sim ('B20_ATMD_NoEarthquake')
figure
CUSUM_Earthquake_NoMeas_NoA = CUSUM;
stairs(CUSUM_Earthquake_NoMeas_NoA.time, CUSUM_Earthquake_NoMeas_NoA.data(:, iState), 'LineWidth', 1.5), hold on
stairs(CUSUM_Earthquake_NoMeas_NoA.time, CUSUM_Earthquake_NoMeas_NoA.data(:, iState)*0 + threshold(iState), '--k', 'LineWidth', 2)
xlabel('Time [s]', 'Interpreter', 'latex'), ylabel('$S_{24}$', 'Interpreter', 'latex')
ylim([0 threshold(iState)*10])
set(gca,'fontsize',FontSize)
set(gcf,'position',[403   246   500   320])

ttd_Earthquake_NoMeas_NoA = timeToDetect( Alarms );

% FDI no earthquake
EI = 0;
DoS_on = 0; % 0:off, 1:on
FDIon  = 1; % 0: off, 1:on
sim ('B20_ATMD_NoEarthquake')
CUSUM_NoEarthquake_NoMeas_FDI = CUSUM;
generatePlot(CUSUM_NoEarthquake_NoMeas_FDI, D_con, threshold, Alarms, h, t)


%% Auxiliary functions
% Plots the results of attacks detection
function  generatePlot(CUSUM, D_con, threshold, Alarms, h, t)
    FontSize = 14;
    ttd = timeToDetect( Alarms );
    ttd(ttd<0) = nan;
    [tdetect, iState] = min(ttd);
    
    figure
    subplot(212)
    stairs(CUSUM.time, CUSUM.data(:, iState), 'LineWidth', 1.5), hold on
    plot(CUSUM.time, CUSUM.data(:, iState)*0 + threshold(iState), '--k', 'LineWidth', 1)
    xlabel('Time (s)', 'Interpreter', 'latex'), ylabel(['$S_{',int2str(iState), '}$'], 'Interpreter', 'latex')
%     plot([tdetect tdetect],[-2 2],'--b')
    ylim([0 threshold(iState)*10])
    xlim([0 30])
    grid on
    set(gca,'fontsize',FontSize)
    
    subplot(211)
    D_con(1:20,:) = D_con(1:20,:)./h;
    plot(t,D_con(20,:)*100, 'LineWidth', 1.5)
    ylabel('ISD (%)', 'Interpreter', 'latex')
    grid on, hold on
    plot([0 t(end)],[1 1],'--r')
    plot([0 t(end)],[-1 -1],'--r')
    xlim([0 30])
    plot([tdetect tdetect],[-2 2],'--b')
    set(gca,'fontsize',FontSize)
    set(gcf,'position',[403   246   423   550])
end
% Finds the detection time for every sensor
function t = timeToDetect(alarm)
    time = alarm.time;
    sz = size(alarm.data);
    t = zeros(sz(2),1)-1;
    for i = 1:sz(1)
        for j = 1:sz(2)
            if alarm.data(i,j) == 1 && t(j) < 0 
                t(j) = time(i);
            end
        end
    end
end