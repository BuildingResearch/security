%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~  Define Clipped Controller for Sample Semiactive Control Design  ~~~ %
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

% This file defines the clipped controller for the sample semiactive control of the bridge structure. 
% Designers/ Researchers SHOULD implement their own controllers.

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% ---  Load Reduced Order Structural Model  --- 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

fprintf (' Constructing the reduced model \n') 

nst=28;                                                      % number of design model states
fprintf (' Number of reduced model states = %d\n',nst) 

[Czo,Dzo]= CDvec(Yrn,C,D,NN,NNe,NNw,TransE,TransW,Ndof,N_Adof,numeq);

[Cmo,Dmo]= CDvec(Ymn,C,D,NN,NNe,NNw,TransE,TransW,Ndof,N_Adof,numeq);
Cf = [C;Czo;Cmo]; Df = [D;Dzo;Dmo];                          

%[Ades,Bdes,Cdes,Ddes] = state_reduce(A,B,Cf,Df,nst);         % Reduce the full-Order Structural Model
load sysR; Ades=Ar;Bdes=Br;Cdes=Cr;Ddes=Dr;

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% --- Parameters for Controller Define --- %   
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

% q1 	  = 5e7;			        % weighting on accel responses
% r1	  = 1e-4;				    % weighting on control forces
% q1 	  = 1e8;			        % weighting on accel responses 1ro
% r1	  = 1e-6;				    % weighting on control forces
% q1 	  = 2e8;			        % weighting on accel responses 2do
% r1	  = 5e-6;				    % weighting on control forces
q1 	  = 1e10;			        % weighting on accel responses 4to
r1	  = 9e-4;				    % weighting on control forces

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Design Controller  --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

Bdes  = [Bdes(:,1:numeq) Bdes(:,numeq+1:end)*Kdev];          % Account for multiple actuators 
Ddes  = [Ddes(:,1:numeq) Ddes(:,numeq+1:end)*Kdev];

% DEFINE WEIGHTING MATRICES
%Regulated responses:  1. Bearing deformation 
%                      2. Displacement at midspan
%                      3. Acceleration at midspan

Czr = Cdes(size(C,1)+Orn,:);
Dzr = Ddes(size(C,1)+Orn,numeq+1:end);

Q    = q1*eye(size(Czr,1));Q(23,23)=0.1*q1;Q(24,24)=0.1*q1;%Q([17:20],[17:20])=10*q1*eye(4);
R    = r1*eye(nloc);                                 

warning off

% STATE FEEDBACK GAINS
Kgain  = lqry(Ades,Bdes(:,numeq+1:end),Czr,Dzr,Q,R);

% STATE ESTIMATOR
%Measurement response: 1. Deformation between the deck and abutment 
%                      2. Absolute acc at both ends and midspan
fbvec  = size(C,1)+length(Orn)+[1:length(Omn)];
numfb  = length(fbvec);         % The number of measurement vectors

SW	   = 25*eye(numeq);  
SV	   = eye(numfb);
Cfb	   = Cdes(fbvec,:);
Dfb    = Ddes(fbvec,1:numeq); 

Lgain  = lqe2(Ades,Bdes(:,1:numeq),Cfb,SW,SV+Dfb*SW*Dfb',SW*Dfb');

% FORM CONTINUOUS STATE-SPACE CONTROLLER
Ac	= Ades-Lgain*Cfb;
Bc	= [Lgain Bdes(:,numeq+1:end)-Lgain*Ddes(fbvec,numeq+1:end)];
Cc	= -Kgain;
Dc	= zeros(nloc,numfb+nloc);

% ~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Check Stability  --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~ %

% CHECK STABILITY OF CONTROLLER
if (max(real(eig(Ac))) > 0) 
	disp('UNSTABLE CONTROLLER !')
% 	break
end

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Convert to Discrete Form  --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

Tcd=dtint;
[Acd,Bcd,Ccd,Dcd] = c2dm(Ac,Bc,Cc,Dc,Tcd,'zoh');

% CHECK STABILITY OF DISCRETE CONTROLLER
EIGVAL = eig(Acd);
if (max(abs(EIGVAL)) > 1) 
	disp('UNSTABLE CONTROLLER (DISCRETE)!')
% 	break
end

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---   Data for Sensors  --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

Max_dis   = 0.33;               % Maximum Displacement of Displ Sensors [m]
Max_acc   = 9.81;               % Maximum Acceleration of Accel Sensors [m/s^2]

Max_volts  = 10;                % Maximum Voltage of Sensors [Volt]

Gsnsr1 = Max_volts/Max_dis;     % Sensitivity of Displ Sensors [10 V = 0.33 m]                             
Ds1= Gsnsr1*eye(length(Omnd));

Gsnsr2 = Max_volts/Max_acc;     % Sensitivity of Accel Sensors [10 V = 9.81 m/s^2]                             
Ds2= Gsnsr2*eye(length(Omna));

Max_frcs   = 1000e3;            % Maximum Force of Sensors
Max_voltsf  = 10;               % Maximum Voltage of Force Sensors
Gsnsrf = Max_voltsf/Max_frcs;   % Gain of Force Sensor        
Ds3= Gsnsrf*eye(nloc);

% Sensors Gain
Gsnsr  =  diag([diag(Ds1); diag(Ds2);  diag(Ds3)]);

% Set Sensor noise
Dt_snr   = dtint;               % Sample Time for Measurement Noise 
S_noise  = 0.03^2*Dt_snr; 	    % Noise Power Fixed to rms 0.03 Volt 
numv = numfb+nloc;%numfb

rand('seed',123);               % White noise seeds
Seed_snr  = round(100000*rand(numv,1));    
% Seed_snr  = round(100000*rand(40,1));
% ~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% --- Data for Actuators --- % 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~ %

Max_for   = 1000e3;             % Maximum Force in Each Device (N)
Gctrl = Max_for/Max_volts;      % Gain of control devices: voltage to force [10V = 1000kN]

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Data for A/D and D/A Converters  --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

SLu_sns  =  +Max_volts;            % Saturation Upper Level for Input Signal [Volt]
SLl_sns  =  -Max_volts;            % Saturation Lower Level for Input Signal [Volt]

quantize_int = 2*Max_volts / 2^16; % Quantization Interval of 16 bit channel
    
SLu_ctr  =  +Max_volts;            % Saturation Upper Level for Control Signal [Volt]
SLl_ctr  =  -Max_volts;            % Saturation Lower Level for Control Signal [Volt]

Umax = 10;

Bcd 	= Bcd*inv(Gsnsr);		   % Adjust controller matrices accordingly

Ccd 	= Ccd/Gctrl;
Dcd 	= Dcd*inv(Gsnsr)/Gctrl;
