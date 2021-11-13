%% Benchmark 20-story building structure with an ATMD system
clear all;clc;
% close all;
% States: X = [20 disp, 1 AMD disp, 20 vel, 1 AMD vel]
%% Parameters
% Earthquake intensity
EI = 1;
% Max. actuator force
MaxF = 2000e3; 
% AMD mass
miu = 0.02; % Mass ratio between 1% and 5%
% LQR gain
r_lqr = 10^-14;
Max_AMD_Disp=0.5;   % Max. stroke
kstop=4e6;          % Stiffness of stop force
cstop=5e7;          % Damping of stop force

% FDI attack
Max_AMD_Disp=0.5;
FDIon = 0;  % 0: off, 1:on
DoS_on = 0; % 0:off, 1:on
FDI_amp = 1000e5; % miu = 0.02
% FDI_amp = 900e2; % miu  = 0.01
% FDI_amp = 2000e3;
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

%% Controller (LQR example)
R=r_lqr;
Q=1*eye(2*n_amd);
K_lqr=lqr(A,B,Q,R);

%% Simulations to tune the CUSUM parameters
nx = length(A);
nu = size(B);
nu = nu(1);
attack = ones(nu,1);
attack(10:12) = 0;
Ts = 0.01;
nsim = 100;
EI = 1;
for j = 1:nsim
    % Kanai-Tajimi model parameters
    wg1= 2*pi + 2*pi*(10-1)*rand(1,1);    % \in [1*2pi,10*2pi]
    So2= 2;
    zg=0.3 + 0.3*rand(1,1);               % \in [0.3, 0.6]
    p1 = 1;                               
    for nn = 1:length(So2)
        wg=wg1;
        So=So2(nn);
        zwg=2*zg*wg;
        wg2 = wg^2;
    end
    % Changing initial conditions
    ci = -0.2+0.4*rand(nx, 1);
    % Simulation
    out = sim('B20_ATMD_Earthquake.slx');
    % Store residuals
    r = residues.data';
    res(:, :, j) = r;
end
%% Select parameters
tau = 0:0.1:80;
% Compute the discount parameters.
% b_i > E[r]
for i = 1:nx
    discount(i) = mean(res(i,:))*2;
end
% Compute the false alarms with different thresholds \tau
falarm = zeros(nx, length(tau));
for j = 1:length(tau)
    for i = 1:nsim
        [~, fal] = CUSUM( res(:, :, i), length(r), discount, tau(j), nx );
        falarm(:, j) = falarm(:, j) + fal;
    end
end
falarmRate = falarm/(length(r)*nsim)*100;

% Find the threshold such that the false alarm rate is smaller or equal
% to rate
rate = 0.01;
th = zeros(nx,1);
for i = 1:length(tau)-1
    for j = 1:nx
        if falarmRate(j,i) >= rate && falarmRate(j,i+1) <= rate
            th(j) = tau(i+1);
        end
    end
end
for j = 1:nx
    if th(j)==0
        th(j) = tau(end);
    end
end
% Plot false alarm rate as a function of the threshold
figure
semilogy(tau,falarmRate)
ylabel('False alarm rate (%)')
xlabel('Threshold')
ylim([0.01 20])
% xlim([0 0.008])
set(gca,'fontsize',14)
grid on

% Function to compute the CUSUM and number of false alarms per simulation
% It takes as inputs:
%   res: the residues for a simulation
%   t: maximum simulation instant
%   v: discount parameter
%   tau: threshold
%   nx: number of system states
% Outputs:
%   S: CUSUM
%   falarms: number of false alarms
function [S, falarm] = CUSUM(res,t,v,tau, nx)
    S = zeros(nx,t+1);
    falarm = zeros(nx,1);
    for k = 1:t
        for state = 1:nx
            if S(state, k) > tau
                S(state, k+1) = 0;
                falarm(state) = falarm(state)+1;
            else
                S(state, k+1) = max(0, S(state, k) - v(state) + res(state, k) );
            end
        end
    end
end