function [J] = eval_india2(ctrl,eqn,Kdev,k)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~       Calculate Evaluation Criteria for the Highway Bridge       ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                  Benchmark Control Problem for                   ~~~ %
% ~~~             Seismic Response of Highway Overcrossing             ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   City College of New York                       ~~~ %
% ~~~                         October, 2004                            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   coded by:     Ping TAN                         ~~~ %
% ~~~                 Supervised by:  Prof. Anil K. Agrawal            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% EVAL_INDIA Calculates the 21 evaluation criteria defined in the Benchmark
%            problem paper after simulating the control system. the output J 
%            may be a vector, or matrix, depending on selected earthquakes).
%
% Usage:     [J] = eval_india(inpt,ctrl)
%            
%            [J] : evaluation criteria for selected earthquakes
%
%
%            ctrl : type of selected control strategy. In three sample
%            control designs
%                       'PA' for passive control design using nonlinear viscous dampers
%                       'AC' for active control design using hydraulic actuators
%                       'SA' for semi-active control design using MR dampers
%
%            eqn : index of earthquake records 
%                   = 1 N. Palm Springs 
%                   = 2 Chichi
%                   = 3 El Centro
%                   = 4 Northridge
%                   = 5 Turkey
%                   = 6 Kobe
%
%            Kdev : number of devices
%                       

ni = nargin;
no = nargout;
% error(nargchk(1,3,ni));

switch ni
    case 1
        if ischar(ctrl)==0
            error ('The ctrl should be a character array');                      
        end
        eqn = [1:6];
        ndev_loc = ones(1,16); Kdev  = diag(ndev_loc);
    case 2
        if ischar(ctrl)==0
            error ('The ctrl should be a character array');                      
        end
        ndev_loc = ones(1,16); Kdev  = diag(ndev_loc);
    case 3
        if ischar(ctrl)==0
            error ('The ctrl should be a character array');                      
        end
end


load JuII;           % Uncontrolled responses 
Ju = Ju*0+1;        % Remove normalized values

m = 4237544;       % Mass of the benchmark highway bridge
W = m*9.81;        % Weight of the benchmark highway bridge

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% --- Find the Evaluation Criteria for Each Earthquake --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

for i = 1:length(eqn)
    if length(eqn)==1 
        inpt=eqn;
    else
        inpt = eqn(i);
    end
    load(['Bridge_' ctrl '_' num2str(k) '.mat'])
%     load(['.\FDI_attacks/Bridge_' ctrl '_' num2str(k) '.mat'])
    tf = max(t_out);
    dt = t_out(2)-t_out(1);
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    % --- Peak Response Evaluation Criteria (J1-J8) --- %
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    
    % J1: Peak Base Shear 
    
    r11 = [1 2];     % base shear of column 1 in x- and y- directions
    r12 = [3 4];     % base shear of column 2 in x- and y- directions
    shear_base = ye(:,[r11 r12]);
    maxFb      = max(max(abs(shear_base)));
    J(1,inpt)  = maxFb/Ju(1,inpt);
    
    % J2: max. moment at base
    
    r21 = [5 6];     % overturning moment of column 1 in x- and y- directions
    r22 = [7 8];     % overturning moment of column 2 in x- and y- directions
    M_base    = ye(:,[r21 r22]);
    maxMb     = max(max(abs(M_base)));
    J(2,inpt) = maxMb/Ju(2,inpt);
    
    % J3: peak displacement at midspan
    
    r3 = [9 10];     % relative displacement at the midspan of the deck
    Dis_ms    = ye(:,r3);  
    J(3,inpt) = max(max(abs(Dis_ms)))/Ju(3,inpt);
    
    % J4: peak acceleration at midspan
    
    r4 = [13 14];    % absolute acceleration at the midspan of the deck

    Acc_ms    = ye(:,r4);  
    J(4,inpt) = max(max(abs(Acc_ms)))/Ju(4,inpt);
    
    % J5: peak deformation of bearing
    
    r51 = [55:70];   % deformation of bearings at the abutments of the bridge
    r52 = [71:74];   % deformation of bearings at the midspan of the bridge
    Dis_ab    = ye(:,[r51 r52]);  
    J(5,inpt) = max(max(abs(Dis_ab)))/Ju(5,inpt);
    
    % --- Evaluation Criteria for Nonlinear Responses of Bridge --- %
    
    Phiy = 2.4305e-003;     % yielding curvature of bent columns
    My   = 5.0822e7;        % yielding moment of bent columns
    N_d(inpt) = Ju(8,inpt); % plastic connections no. of the uncontrolled bridge
    
    % J6: Ductility Factor 
    
    r6 = [15:54];    % curvature of bent column
    Phi       = ye(:,r6); 
    J(6,inpt) = max(max(abs(Phi)))/ Ju(6,inpt);
    
    % J7: Dissipated Energy of the Curvatures at the End of Members 
    
    r7=[95:134];
    Ei = ye(:,r7);
    if N_d(inpt) > 0
        J(7,inpt) = max(max(Ei)) / Ju(7,inpt);
    else
        J(7,inpt) = 0;
    end
    
    % J8: Ratio of Plastic Connections
    
    if N_d(inpt)>0
        J(8,inpt) = length(find(max(abs(Phi))/Phiy>1.0)) / Ju(8,inpt);
    else
        J(8,inpt) = 0;
    end
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    % ---  Normed Response Evaluation Criteria (J9-J14)  --- %
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    
    % J9: Normed Base Shear 
    
    J(9,inpt) = max(sqrt((1/tf)*sum(shear_base.*shear_base)*dt)) / Ju(9,inpt);
    
    % J10: Normed Moment at Base
    
    J(10,inpt) = max(sqrt((1/tf)*sum(M_base.*M_base)*dt)) / Ju(10,inpt);
    
    % J11: Normed Displacement at Midspan
    
    J(11,inpt) = max(sqrt((1/tf)*sum(Dis_ms.*Dis_ms)*dt)) / Ju(11,inpt);
    
    % J12: peak acceleration at midspan
    
    J(12,inpt) = max(sqrt((1/tf)*sum(Acc_ms.*Acc_ms)*dt)) / Ju(12,inpt);
    
    % J13: peak deformation of bearing
    
    J(13,inpt) = max(sqrt((1/tf)*sum(Dis_ab.*Dis_ab)*dt)) / Ju(13,inpt);
    
    % J14: Normed Basis Ductility Factor
    
    J(14,inpt) = max(sqrt((1/tf)*sum(Phi.*Phi)*dt)) / Ju(14,inpt);
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    % --- Control Strategy Evaluation Criteria (J15-J21) --- %
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    
    r16 = [1:20];
    Dis_dev = yc(:,r16);
    r17 = [21:40];
    Vel_dev = yc(:,r17);
    
    yf = yf(:,r16);
    Pow_dev = [abs(Vel_dev.*yf*Kdev)]';                 % device power
    
    % Peak Control Force
    
    J(15,inpt) = max(max(abs(yf))) / W;
    
    % Peak Control Device Stroke
    
    J(16,inpt) = max(max(abs(Dis_dev))) / x_max(inpt);
    
    % Peak Control Power
    
    J(17,inpt) = max(sum(Pow_dev))/ W / x_dmax(inpt);
    
    % Total Control Power
    
    J(18,inpt) = max(sum(Pow_dev).*dt)/ W / x_max(inpt);
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    % --- Control Strategy Evaluation Criteria (J15-J21) --- %
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
    
    % Number of Control Devices
    
    J(19,inpt) = size(yf,2);
    
    % Number of Required Sensors
    
    if ctrl(1:2)=='SA'		% Obtained previously from design
        J(20,inpt) = 36;
    else
        J(20,inpt) = 16;		
    end
    
    % Computational Resourse
    
    J(21,inpt) = 28;			% Obtained previously from design   
    
    % CHECK DEVICE CAPACITY 
    MAXF = max(max(abs(yf))); 
    if (MAXF > 1.0e6), warning('Control force exceeded the device capacity!');end
end

if ctrl(1:2) == 'PA' 
    J([17 18 20 21],eqn) = zeros(4,size(eqn,2));
elseif ctrl(1:2) == 'SA'   % control power associated with semiactive control strategies are not considered
    J([17 18],eqn) = nan*ones(2,size(eqn,2));
end

error(nargoutchk(0,1,no));   
switch no
    case 0
        if length(eqn)==1
            % ~~~~~~~~~~~~~~~~~~~~~~~~ %
            % ---  DISPLAY OUTPUTS --- %
            % ~~~~~~~~~~~~~~~~~~~~~~~~ %
            JJ=J(:,inpt); 
            
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Base Shear\n'],[1 JJ(1)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Base Moment\n'],[2 JJ(2)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Midspan Displacement\n'],[3 JJ(3)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Midspan Acceleration\n'],[4 JJ(4)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Abutment Displacement\n'],[5 JJ(5)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Ductility\n'],[6 JJ(6)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Dissipated Energy\n'],[7 JJ(7)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Plastic Connections\n'],[8 JJ(8)])
%             
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Base Shear\n'],[9 JJ(9)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Base Moment\n'],[10 JJ(10)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Midspan Displacement\n'],[11 JJ(11)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Midspan Acceleration\n'],[12 JJ(12)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Abutment Displacement\n'],[13 JJ(13)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Ductility\n'],[14 JJ(14)])
%             
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Control Force\n'],[15 JJ(15)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Device Stroke\n'],[16 JJ(16)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Power\n'],[17 JJ(17)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Total Power\n'],[18 JJ(18)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Number Devices\n'],[19 JJ(19)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Number Sensors\n'],[20 JJ(20)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Control Resources\n'],[21 JJ(21)])    
            save(['J_' ctrl '_' num2str(inpt) '.mat'],'J')
        else 
            save(['J_' ctrl '.mat'],'J')           
        end
        clear all
    case 1
        if length(eqn)==1
            JJ=J(:,inpt);             
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Base Shear\n'],[1 JJ(1)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Base Moment\n'],[2 JJ(2)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Midspan Displacement\n'],[3 JJ(3)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Midspan Acceleration\n'],[4 JJ(4)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Bearing Deformation\n'],[5 JJ(5)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Ductility\n'],[6 JJ(6)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Dissipated Energy\n'],[7 JJ(7)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Plastic Connections\n'],[8 JJ(8)])
%             
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Base Shear\n'],[9 JJ(9)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Base Moment\n'],[10 JJ(10)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Midspan Displacement\n'],[11 JJ(11)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Midspan Acceleration\n'],[12 JJ(12)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Abutment Displacement\n'],[13 JJ(13)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Norm Ductility\n'],[14 JJ(14)])
%             
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Control Force\n'],[15 JJ(15)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Device Stroke\n'],[16 JJ(16)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Max Power\n'],[17 JJ(17)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Total Power\n'],[18 JJ(18)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Number Devices\n'],[19 JJ(19)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Number Sensors\n'],[20 JJ(20)])
%             fprintf(1,['%3.0f\t%4.5e\t%% ','  Control Resources\n'],[21 JJ(21)])    
            save(['output/J_' ctrl '_' num2str(inpt) '.mat'],'J')            
        elseif length(eqn)==6
            [J] = J
            save(['J_' ctrl '.mat'],'J')
        else
            save(['J_' ctrl '.mat'],'J')            
        end
end
