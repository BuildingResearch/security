%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~                           READ INPUT                             ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                  Benchmark Control Problem for                   ~~~ %
% ~~~               Seismic Response of Highway Crossing               ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   City College of New York                       ~~~ %
% ~~~                         October, 2004                            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   coded by:     Ping TAN                         ~~~ %
% ~~~                 Supervised by:  Prof. Anil K. Agrawal            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This program defines structural data, earthquake records and general simulation data.
% Designers/Researchers MAY change the integration step and the convergence tolerance.  

% METRIC SYSTEM : N - m- s

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% --- Define Structural Data for Benchmark Bridge --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

% fprintf('Loading Structural DATA\n')

% load structural array of benchmark bridge
load HWbridge_dataII.mat;              

Cs = HWbridge_data.C;                  % damping matrix
Ks = HWbridge_data.K;                  % initial stiffness matrix 
Ms = HWbridge_data.M;                  % mass matrix
KL = HWbridge_data.KL;                 % linear part of the global stiffness matrix
I1 = HWbridge_data.I1;                 % coefficient of ground motion
EL_bear = HWbridge_data.EL_bear;       % bearing element no.
EL_iso = HWbridge_data.EL_iso;         % bearing element no.
EL_column0 = HWbridge_data.EL_column0; % column element no.
column=HWbridge_data.column;           % column stiffness structure 
bear=HWbridge_data.bear;               % bearing stiffness structure
bearm=HWbridge_data.bearm;             % plastic stiffness matrix of bearing element ??
Lxy = HWbridge_data.Lxy;               % location of nodes of deck-ends and abutments
TRAN = HWbridge_data.TRAN;             % transformation matrix
TT_column = HWbridge_data.TT_column;   % directional cosine of bent column
elem = HWbridge_data.elem;             % structure of element stiffness and mass matrices of linear members
N_Adof = HWbridge_data.N_Adof;         % no. of active DOF
nne = HWbridge_data.nne;               % constrained DOF
nu = HWbridge_data.nu;                 % 

NN=elem.NN;                            % nodes of bridge FEM model
NNe=elem.NNe;                          % node number of east deck-end and abutment
NNw=elem.NNw;                          % node number of west deck-end and abutment

MP = HWbridge_data.MP;                 % yielding moment of bent column
L = HWbridge_data.L;                   % length of column segment 
EI = HWbridge_data.EI;                 % pre-yield flexure stiffness of column
EI1 = HWbridge_data.EI1;               % post-yield flexure stiffness of column
p = HWbridge_data.p;                   % stiffness ratio       
q = HWbridge_data.q;                   
yP = HWbridge_data.yP;                 % yield displacement of bearing
yPm= HWbridge_data.yPm;
kb1 = HWbridge_data.kb1;kb1m= HWbridge_data.kb1m;               % pre-yield shear stiffness of bearing
kb2 = HWbridge_data.kb2;kb2m= HWbridge_data.kb2m;               % post-yield shear stiffness of bearing

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---     Read Earthquake Data     --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

eq_name={'Npalmspr' 'Chichi084' 'Elcentro' 'Rinadi' 'TurkBolu' 'Kobe_NIS'};

for i=1:size(eq_name,2)
    load(eq_name{i})
    eq(1,1,i).dt   = dt;
    eq(1,1,i).Tf   = Tf;
    eq(1,1,i).data = data;    
    eq(1,1,i).name = eq_name{i};    
end

intensity = 1.0;                       % intensity of earthquake 
direc = 3;                             % direction of input earthquake (1: x-dir; 2: y-dir; 3: bi-dir)
numeq  = 2;                            % The number of earthquake input dirctions
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---    Define General Data for Simulation    --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

if ctrl_type == 2             
    dtint = 0.005;                     % for LQG controller
else
    dtint = 0.002;                     % for MR or Nonlinear Viscous Damper
end
solver.alfa  = 1/4;                    % alfa  value for Newmark-beta Method
solver.delta = 1/2;                    % delta value for Newmark-beta Method
solver.TOL = MP*1e-2;                  % specified moment tolerance of bent columns for nonlinear analysis
solver.TOLb = yP*1e-2;                 % specified displacement tolerance of bearings at abutment for nonlinear analysis
solver.TOLbm = yPm*1e-2;               % specified displacement tolerance of bearings at midspan for nonlinear analysis
solver.dtint = dtint;                  % integration time step

% fprintf(['Integration Time Step = %4.3e \n'],[dtint])    
