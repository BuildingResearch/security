%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~                           DEVICE DATA                            ~~~ %
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


% Parameters and locations of sample Control Systems are specified in this m-file.

% NOTE: This file is user-changeable. Participants in the benchmark study should define 
% their own control systems. The number and locations of control devices can also been 
% changed appropriately to implement their control designs. 

% ~~~~~~~~~~~~~~~~~~~~~~~ %
% --- Device Location --- %
% ~~~~~~~~~~~~~~~~~~~~~~~ %

% NOTE: Devices with 2 connection points require 6 columns
% per row, and devices with 1 connection point require 3
% columns per row. For a combination of these cases, use 6 columns
% with zeros to fill the last three columns. 

%Device#       node#       dof#        factor       
Dev = [       
  1            101            1           1                           
  1            105            1          -1             
  2            101            2           1                           
  2            105            2          -1             
  3            102            1           1                           
  3            107            1          -1             
  4            102            2           1                           
  4            107            2          -1             
  5            103            1           1                           
  5            110            1          -1             
  6            103            2           1                           
  6            110            2          -1             
  7            104            1           1                           
  7            112            1          -1             
  8            104            2           1                           
  8            112            2          -1   
  9            113            1           1                           
  9            117            1          -1             
 10            113            2           1                           
 10            117            2          -1             
 11            114            1           1                           
 11            119            1          -1             
 12            114            2           1                           
 12            119            2          -1              
 13            115            1           1                           
 13            122            1          -1             
 14            115            2           1                           
 14            122            2          -1             
 15            116            1           1                           
 15            124            1          -1             
 16            116            2           1                           
 16            124            2          -1    
 17             33            1           1                           
 17             43            1          -1             
 18             33            2           1                           
 18             43            2          -1       
 19             54            1           1                           
 19             44            1          -1             
 20             54            2           1                           
 20             44            2          -1       
      ];

% Coefficient matrix for Control Forces
Ndof = size(Ms,1); 
nloc = Dev(end,1);
b = zeros(Ndof,nloc);

Lexy = Lxy(1:size(Lxy,1)/2,:);
Lwxy = Lxy(size(Lxy,1)/2+1:end,:);
TransE = TRAN(1:size(TRAN,1)/2,:); 
TransW = TRAN(size(TRAN,1)/2+1:end,:); 

for j=1:4
TransE0(2*j-1:2*j,:) = TransE(3*j-2:3*j-1,:);
TransW0(2*j-1:2*j,:) = TransW(3*j-2:3*j-1,:);
end

% account for rigid links of both deck-ends and abutments 
%                                 node#    dof# (  x   y  Rz)
% mass center of east deck-end      1           (  1   2   5) 
% mass center of east abutment    125           (425 426 427)
% mass center of west deck-end     32           (181 182 184) 
% mass center of west abutment    126           (428 429 430)

% East deck-end 
    b(1,1) = 1; b(1,3) = 1; b(1,5) = 1; b(1,7) = 1;                     % x-direction
    b(2,2) = 1; b(2,4) = 1; b(2,6) = 1; b(2,8) = 1;                     % y-direction
    b(5,1) = b(1,1)*(-Lexy(1,2)); b(5,2) = b(2,2)*Lexy(1,1);            % rotation
    b(5,3) = b(1,3)*(-Lexy(2,2)); b(5,4) = b(2,4)*Lexy(2,1);
    b(5,5) = b(1,5)*(-Lexy(3,2)); b(5,6) = b(2,6)*Lexy(3,1);
    b(5,7) = b(1,7)*(-Lexy(4,2)); b(5,8) = b(2,8)*Lexy(4,1);
% East abutment    
    b(425,1) = -1; b(425,3) = -1; b(425,5) = -1; b(425,7) = -1;         % x-direction
    b(426,2) = -1; b(426,4) = -1; b(426,6) = -1; b(426,8) = -1;         % y-direction
    b(427,1) = b(425,1)*(-Lexy(5,2)); b(427,2) = b(426,2)*Lexy(5,1);    % rotation
    b(427,3) = b(425,3)*(-Lexy(7,2)); b(427,4) = b(426,4)*Lexy(7,1);
    b(427,5) = b(425,5)*(-Lexy(10,2)); b(427,6) = b(426,6)*Lexy(10,1);
    b(427,7) = b(425,7)*(-Lexy(12,2)); b(427,8) = b(426,8)*Lexy(12,1);

% West deck-end 
    b(180,9) = 1; b(180,11) = 1; b(180,13) = 1; b(180,15) = 1;          % x-direction
    b(181,10)= 1; b(181,12) = 1; b(181,14) = 1; b(181,16) = 1;          % y-direction
    b(184,9) = b(180,9)*(-Lwxy(1,2));  b(184,10) = b(181,10)*Lwxy(1,1); % rotation
    b(184,11) =b(180,11)*(-Lwxy(2,2)); b(184,12) = b(181,12)*Lwxy(2,1);
    b(184,13) =b(180,13)*(-Lwxy(3,2)); b(184,14) = b(181,14)*Lwxy(3,1);
    b(184,15) =b(180,15)*(-Lwxy(4,2)); b(184,16) = b(181,16)*Lwxy(4,1);
% West abutment    
    b(428,9) = -1; b(428,11) = -1; b(428,13) = -1; b(428,15) = -1;      % x-direction
    b(429,10)= -1; b(429,12) = -1; b(429,14) = -1; b(429,16) = -1;      % y-direction
    b(430,9) = b(428,9)*(-Lwxy(5,2));  b(430,10) = b(429,10)*Lwxy(5,1); % rotation
    b(430,11)= b(428,11)*(-Lwxy(7,2)); b(430,12) = b(429,12)*Lwxy(7,1);
    b(430,13)= b(428,13)*(-Lwxy(10,2)); b(430,14) = b(429,14)*Lwxy(10,1);
    b(430,15)= b(428,15)*(-Lwxy(12,2)); b(430,16) = b(429,16)*Lwxy(12,1);    
% Midspan
    b(185,17) = 1; b(431,17) = -1; b(299,19) = 1; b(437,19) = -1;       % x-direction
    b(186,18) = 1; b(432,18) = -1; b(300,20) = 1; b(438,20) = -1;       % x-direction

Yecm.b=b; 

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Parameters for control devices  --- %   
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

ndev_loc = 1*ones(nloc,1);         % number of devices at each location 
ndev  = sum(ndev_loc);		       % total number of control devices
nloc  = length(ndev_loc);          % number of locations for devices
Kdev  = diag(ndev_loc);         

% Passive 

cv = 1e5;                          % damping coefficient of nonlinear viscous dampers
av = 0.6;                          % power of nonlinear viscous dampers
% Active

% Semiactive
                                   % Parameters for MR dampers
alph_a = 1.0872e07;  
alph_b = 4.9616e07;  

c0_a = 4.40e2;       
c0_b = 44.0e2;       

AA = 1.2;
GG = 300;      
BB = 300;      

eta=50;            
A1=[-eta*eye(nloc)];B1=[eta*eye(nloc)];;C1=eye(nloc);D1=zeros(nloc,nloc);
