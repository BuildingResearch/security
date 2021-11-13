function [A,B,C,D] = SYS_HWB(Ms,Ks,Cs,b,I1,nloc)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~     BUILDING THE STATE SPACE SYSTEM MATRICES FOR THE BRIDGE      ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                  Benchmark Control Problem for                   ~~~ %
% ~~~               Seismic Response of Highway Bridge               ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   City College of New York                       ~~~ %
% ~~~                         October, 2004                            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   coded by:     Ping TAN                         ~~~ %
% ~~~                 Supervised by:  Prof. Anil K. Agrawal            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 

% SYS_HWBE     Builds the full-state space representation of the structure

% 
% Usage:       [A,B,C,D] = SYS_HWB(Ms,Ks,Cs,b,I1,nloc);
%
% Inputs:
%        Ms    Mass matrix  
%        Ks    Initial linear elastic stiffness matrix
%        Cs    Damping matrix 
%        b     Device location vector
%        I1    ground acceleration vector
%      nloc    number of device locations 
% Outputs:     A,B,C,D matrices: State space representation of the
%              structure. 
%
%  METRIC SYSTEM : N - m- s


Ndof=size(Ms,1);
% fprintf ('DOF Number of Evaluation Model = %d\n',Ndof) 

invM=inv(Ms);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Build The State Space System Matrices %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf (' Constructing the System for Benchmark Highway Bridge \n') 

% ~~~~~~~~~~~~~~~~~ %
% ---  A Matrix --- %
% ~~~~~~~~~~~~~~~~~ %

% fprintf('   A matrix\n')

A = [  spalloc(Ndof,Ndof,0)   speye(Ndof)
      -invM*Ks                -invM*Cs  ];

% ~~~~~~~~~~~~~~~~~ %
% ---  B Matrix --- %
% ~~~~~~~~~~~~~~~~~ %

% fprintf('   B matrix\n')

% Coefficient matrix for earthquake excitation

E = [spalloc(Ndof,2,0);-I1];

% Coefficient matrix for Control Commands

Bu = [spalloc(Ndof,nloc,0); invM*b];

% Complete B matrix 
B = [E Bu];

% ~~~~~~~~~~~~~~~~~ %
% ---  C Matrix --- %
% ~~~~~~~~~~~~~~~~~ %
% Output = Relative Displacements
%          Relative Velocities
%          Absolute accelerations

% fprintf('   C matrix\n')

C = [ speye(Ndof) spalloc(Ndof,Ndof,0); A];

% ~~~~~~~~~~~~~~~~~ %
% ---  D Matrix --- %
% ~~~~~~~~~~~~~~~~~ %

% fprintf('   D matrix\n\n')

% Ground excitation 

F = [spalloc(size(C,1),2,0)];

% Control Commands

Du = [spalloc(Ndof,size(Bu,2),0); Bu];

% Complete D matrix

D = [F Du];