%% Highway Bridge with 20 MR-dampers under a DoS attack
% Authors: Alejandro Palacio-Betancur, Luis Burbano
% Date: August 21, 2021
% Project: Cyber attacks on civil infrastructures

clear all, close all
addpath('AuxiliarFunctions', 'data', 'output')
%% DoS design
% Load reduced-order model
load('BridgeMR20LQG.mat')
System = ss(A, B, C, Db);

% Optimization parameters
n_actuators = size(B);
n_actuators = n_actuators(2);
nx = length(A);
li = zeros(1,n_actuators);
ls = ones(1,n_actuators);
Ae = [ones(1,n_actuators); -ones(1,n_actuators)];
Be = 5;

n_actuators = 20;
IntCon = 1:n_actuators;

% Optimize or use the saved data
OPTIMIZE = 1;
if OPTIMIZE
    opts = optimoptions(@ga,'UseParallel', true, 'UseVectorized', false, ...
                        'MaxGenerations', 20); %Options for optimization
    % GA optimization
    for k=0:n_actuators
        fprintf('Optimization %i\n', k)
        Be = [k;-k];
        [Selopt, cost] = ga(@(x)funobjDoS(x,A, B, eye(nx), B*0, E, -Kgain), n_actuators, Ae,Be, [],[], li, ls, [], IntCon, opts );
    
        costs(k+1) = cost;
        Selection(k+1,:) = Selopt;
    end
else
    load('optimalAttacks.mat')    % Alternatively, load costs and Selection used in the paper 
end
%% Bridge simulation 
% Control type
ctrl_type = 3;  %semi-active

% Earthquake
%EQNUM:  'Npalmspr', 'Chichi084', 'Elcentro', 'Rinadi', 'TurkBolu', 'Kobe_NIS'
EQNUM = 6; 

% Model
eval('readinp')
eval('SYS_IO')
[A,B,C,D]=SYS_HWB(Ms,Ks,Cs,b,I1,nloc);
ctrl='DoS';
eval('samp_MR');                    % Controller design
Sim_model='HWbridge_S_DoS';    % Simulink model

% Simulation
dt_out = 0.02; 
Num_decimate = round(dt_out/dtint); % Decimation 

% Set Simulink integration options
OPTIONS = simset('solver','ode5','FixedStep',dtint);
if EQNUM == 7
    inpnum = [1:6];
else
    inpnum = EQNUM;
end

% Controlled structured and 20 attacks
for i = 1:n_actuators + 1 
    fprintf('Simulation %i\n', i)
    % DoS_s vector: 0 to desactivate actuator
    if i < n_actuators + 1
        j = i+1;
        DoS_s = Selection(i,:);     % DoS_s = 0: attack off
                                    % DoS_s = 1: attack on
    else
        j = 1;
        DoS_s = Selection(1,:)*0;   % DoS_s = 0: attack off
                                    % DoS_s = 1: attack on
    end
    
    % Earthquake load
    inpt = inpnum(1);
    t = eq(1,1,inpt).data(:,1);
    if direc == 1                                      % Earthquake in the x direction
        ground = [eq(1,1,inpt).data(:,2) zeros(length(t),1)]; 
    elseif direc == 2                                  % Earthquake in the y direction
        ground = [zeros(length(t),1) eq(1,1,inpt).data(:,3)];
    else                                               % Earthquake in the bi-direction
        ground = eq(1,1,inpt).data(:,[2 3]);
    end
    ground = ground*9.81;                              % in m/s^2  
    MMm = full(Ms);KKm = full(Ks);CCm = full(Cs);KL = full(KL);                   
    tf = eq(1,1,inpt).Tf;                              % Duration of selected earthquake
    
    % Simulation
    sim(Sim_model,[0 tf],OPTIONS,[])
    save(['output/Bridge_' ctrl '_' num2str(j - 1) '.mat'],'ye','yc','yf','t_out','dtint')
    
    % Evaluation Criteria 
    J = eval_india2(ctrl, inpnum, Kdev, j - 1);
    J3(j) = J(3, end);
end

%% Figures
% H2 norm and mid-span displacement for k=1,..,20
DoS_plot(20, J3(2:end), costs)

% Histogram of attacked actuators
DoS_HistogramPlot

% Time-history comparison 
load('Bridge_DoS_1.mat')
Disp_test(1, :) = ye(:, 10)';
load('Bridge_DoS_7.mat')
Disp_test(2, :) = ye(:, 10)';
load('Bridge_DoS_rand.mat')
Disp_test(3, :) = ye(:, 10)';
t=t_out;
At_test=zeros(3,20);
load('optimalAttacks.mat')
At_test(2, :) = Selection(7, :);
At_test(3, [3 4 13 14 7 19]) = 1;
DoS_comp_plot(At_test,Disp_test, t, 6, 20)
