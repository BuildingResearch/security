function [Co,Do,Fo,Duo]= CDvec(Yn,C,D,NN,NNe,NNw,TransE,TransW,Ndof,N_Adof,numeq)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~     BUILDING THE STATE SPACE SYSTEM MATRICES FOR THE BRIDGE      ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                  Benchmark Control Problem for                   ~~~ %
% ~~~               Seismic Response of Highway Bridge                 ~~~ %
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

% CDvec        Builds the state space representation of the structure
%              with the sensors, actuators, regulated response, or connections defined in
%              SYS_IO.m
% 
% Usage:       [Co,Do,Fo,Duo]= CDvec(Yn,C,D,NN,NNe,NNw,TransE,TransW,Ndof,N_Adof,numeq)
%
% Inputs:
%        Yn     User Defined Input-Outputs DATA, e.g. Ymn, Yrn, etc  
%       C,D     Full-state output matrix, developed in SYS_HWB.m
%   NN,NNe,NNw  Node data    
% TRansE,TransW Transformation matrices of both deck-ends and abutments
%        Ndof   No. of DOF of the evaluation model
%      N_Adof   No. of active DOF
%       numeq   The number of earthquake input dirctions
%
% Outputs:      Co,Do matrices: developed State space representation of the
%              structure with the sensors, actuators and connections 
%
%  METRIC SYSTEM : N - m- s      

nd=find(Yn(:,5)==1);  Yn_d = Yn(nd,:);     
nv=find(Yn(:,5)==2);  Yn_v = Yn(nv,:);     
na=find(Yn(:,5)==3);  Yn_a = Yn(na,:);

%~~~~~~~~~~~~~~~~~~~~%
%--- Displacement ---%
%~~~~~~~~~~~~~~~~~~~~%

nd1=[];    d1=[];                   % relative displacement between two nodes
for j = 2:size(Yn_d,1)
    if Yn_d(j,1)==Yn_d(j-1,1) & Yn_d(j,4)==-Yn_d(j-1,4)
        d1 = [d1 Yn_d(j,1)]; nd1=[nd1;find(Yn(:,1)==Yn_d(j,1))];
    end
end

d2=[];                             % relative displacement to ground
for j = 1:length(nd1)
    d2=[d2; find(nd==nd1(j))];
end
nd2=nd;nd2(d2)=[];

% disp. vec1
vecd1 = Yn(nd1,:);  
Cd1 = [spalloc(size(vecd1,1)/2,2*Ndof,0)];

Tee = [spalloc(3,Ndof,0)]; Tee(1,1)=1;Tee(2,2)=1;Tee(3,5)=1;
Tea = [spalloc(3,Ndof,0)]; Tea(1,425)=1;Tea(2,426)=1;Tea(3,427)=1;
Twe = [spalloc(3,Ndof,0)]; Twe(1,180)=1;Twe(2,181)=1;Twe(3,184)=1;
Twa = [spalloc(3,Ndof,0)]; Twa(1,428)=1;Twa(2,429)=1;Twa(3,430)=1;

for j=1:size(vecd1,1)
    N_lab= vecd1(j,2);
    nde=find(NNe==N_lab); ndw=find(NNw==N_lab);
    if  length(nde)+length(ndw)==0
        n1 = NN(find(NN(:,1)==N_lab),5)+vecd1(j,3)-1;
        Cdn1=find(N_Adof==n1);
        Cd1(ceil(j/2),Cdn1)=vecd1(j,4);   
    elseif length(ndw)==0
        if nde <= 4
            Cd1(ceil(j/2),:) = Cd1(ceil(j/2),:)+TransE(3*(nde-1)+vecd1(j,3),:)*[Tee spalloc(3,Ndof,0)]*vecd1(j,4); 
        else
            Cd1(ceil(j/2),:) = Cd1(ceil(j/2),:)+TransE(3*(nde-1)+vecd1(j,3),:)*[Tea spalloc(3,Ndof,0)]*vecd1(j,4); 
        end
    else
        if ndw <= 4
            Cd1(ceil(j/2),:) = Cd1(ceil(j/2),:)+TransW(3*(ndw-1)+vecd1(j,3),:)*[Twe spalloc(3,Ndof,0)]*vecd1(j,4); 
        else
            Cd1(ceil(j/2),:) = Cd1(ceil(j/2),:)+TransW(3*(ndw-1)+vecd1(j,3),:)*[Twa spalloc(3,Ndof,0)]*vecd1(j,4); 
        end
     end
 end    
    
% dis vec2
vecd2 = Yn_d(nd2,:);  
Cd2 = [spalloc(size(vecd2,1),2*Ndof,0)];

for j=1:size(vecd2,1)
    N_lab=vecd2(j,2);
    n1=NN(find(NN(:,1)==N_lab),5)+vecd2(j,3)-1;
    Cdn2=find(N_Adof==n1);
    Cd2(j,Cdn2)=vecd2(j,4);
end
Cd = [Cd1; Cd2];

%~~~~~~~~~~~~~~~~%
%--- Velocity ---%
%~~~~~~~~~~~~~~~~%

nv1=[];    v1=[];                   % relative velocity between two nodes
for j = 2:size(Yn_v,1)
    if Yn_v(j,1)==Yn_v(j-1,1) & Yn_v(j,4)==-Yn_v(j-1,4)
        v1 = [v1 Yn_v(j,1)]; nv1=[nv1;find(Yn(:,1)==Yn_v(j,1))];
    end
end

v2=[];                              % relative velocity to ground
for j = 1:length(nv1)
    v2=[v2; find(nv==nv1(j))];
end
nv2=nv;nv2(v2)=[];

% velo. vec1
vecv1 = Yn(nv1,:);  
Cv1 = [spalloc(size(vecv1,1)/2,2*Ndof,0)];

Tee = [spalloc(3,Ndof,0)]; Tee(1,1)=1;Tee(2,2)=1;Tee(3,5)=1;
Tea = [spalloc(3,Ndof,0)]; Tea(1,425)=1;Tea(2,426)=1;Tea(3,427)=1;
Twe = [spalloc(3,Ndof,0)]; Twe(1,180)=1;Twe(2,181)=1;Twe(3,184)=1;
Twa = [spalloc(3,Ndof,0)]; Twa(1,428)=1;Twa(2,429)=1;Twa(3,430)=1;

for j=1:size(vecv1,1)
    N_lab= vecv1(j,2);
    nve=find(NNe==N_lab); nvw=find(NNw==N_lab);
    if  length(nve)+length(nvw)==0
        n1 = NN(find(NN(:,1)==N_lab),5)+vecv1(j,3)-1;
        Cvn1=find(N_Adof==n1);
        Cv1(ceil(j/2),Cvn1)=vecv1(j,4);   
    elseif length(nvw)==0
        if nve <= 4
            Cv1(ceil(j/2),:) = Cv1(ceil(j/2),:)+TransE(3*(nve-1)+vecv1(j,3),:)*[Tee spalloc(3,Ndof,0)]*vecv1(j,4); 
        else
            Cv1(ceil(j/2),:) = Cv1(ceil(j/2),:)+TransE(3*(nve-1)+vecv1(j,3),:)*[Tea spalloc(3,Ndof,0)]*vecv1(j,4); 
        end
    else
        if nvw <= 4
            Cv1(ceil(j/2),:) = Cv1(ceil(j/2),:)+TransW(3*(nvw-1)+vecv1(j,3),:)*[Twe spalloc(3,Ndof,0)]*vecv1(j,4); 
        else
            Cv1(ceil(j/2),:) = Cv1(ceil(j/2),:)+TransW(3*(nvw-1)+vecv1(j,3),:)*[Twa spalloc(3,Ndof,0)]*vecv1(j,4); 
        end
     end
 end    

% Velo. vec2
vecv2 = Yn_v(nv2,:);  
Cv2 = [spalloc(size(vecv2,1),2*Ndof,0)];

for j=1:size(vecv2,1)
    N_lab=vecv2(j,2);
    n1=NN(find(NN(:,1)==N_lab),5)+vecv2(j,3)-1;
    Cvn2=find(N_Adof==n1);
    Cv2(j,Cdn2)=vecv2(j,4);
end
Cv = [Cv1; Cv2];

%~~~~~~~~~~~~~~~~~~~~%
%--- Acceleration ---%
%~~~~~~~~~~~~~~~~~~~~%

% absolute acceleration vector
Du = D(:,numeq+1:end);
Ca  = [spalloc(size(na,1),2*Ndof,0)];
Ca0 = C(2*Ndof+1:3*Ndof,:);
Da0 = Du(2*Ndof+1:3*Ndof,:); 

Cna=[];
for j=1:size(Yn_a,1)
    N_lab = Yn_a(j,2);
    n1=NN(find(NN(:,1)==N_lab),5)+Yn_a(j,3)-1;
    Cna=[Cna find(N_Adof==n1)];
end
Ca=Ca0(Cna,:);

%%%%%%%%%%%%%%%%%%%%%%%%%
%   Complete C matrix   %
%%%%%%%%%%%%%%%%%%%%%%%%%
Co = [Cd;Cv;Ca];

%%%%%%%%%%%%%%%%%%%%%%%%%
%   Complete D matrix   %
%%%%%%%%%%%%%%%%%%%%%%%%%

Fo = [spalloc(size(Co,1),2,0)];

Dd = spalloc(size(Cd,1),size(Du,2),0);
Dv = spalloc(size(Cv,1),size(Du,2),0);

Da  = [spalloc(size(Ca,1),size(Du,2),0)];
Da0 = Du(Ndof*2+1:3*Ndof,:);Da  = Da0(Cna,:);

Duo = [Dd;Dv;Da];

Do = [Fo Duo];
