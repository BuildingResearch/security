function [sys,x0,str,ts] = NLHWbridgeII(t,x,u,flag,HWbridge_data,solver,Yecm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~                         M-FILE S FUNCTION                        ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                  Benchmark Control Problem for                   ~~~ %
% ~~~               Seismic Response of Highway Crossing               ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   City College of New York                       ~~~ %
% ~~~                         October, 2004                            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   coded by:     Ping Tan                         ~~~ %
% ~~~                 Supervised by:  Prof. Anil K. Agrawal            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NLHWbridge    performs the nonlinear calculations for the simulator.

% Usage:          [sys,x0,str,ts] = NLHWbridge(t,x,u,flag,HWbridge_data,solver,Yecm)
%
% INPUTS:
% Standard block parameters: t,x,u,flag
%                           t         current time
%                           x         state vector
%                           u         input of the block
%                           flag      indicates a task to be performed
% Additional S-function parameters: HWbridge_data, solver, b, Ycn

%                           HWbridge_data   structural data of the bridge model
%                           solver          data for Newmark-beta Method
%                           Yecm            structural data for Ye, Yc and Ym

% OUTPUTS
% sys             generic return argument, depending on the flag values. 
% x0              The initial state values. x0 is usually ignored, except
%                 when flag=0.
% str             reserved for future use. The default is set to the empty
%                 matrix, [].
% ts,             two-columnmatrix containing the sample times and offsets
%                 of the block
%
switch flag,
    
    %==================%
    %  Initialization  %
    %==================%
    case 0,
        [sys,x0,str,ts] = mdlInitializeSizes(t,x,u,HWbridge_data,solver,Yecm);
        
        %==========%
        %  Update  %
        %==========%
    case 2,                                                
        sys = mdlUpdate(t,x,u,HWbridge_data,solver,Yecm);
        
        %==========%
        %  Output  %
        %==========%
    case 3,                                                
        sys = mdlOutputs(t,x,u,HWbridge_data,solver,Yecm);
        
        %==========%
        % Terminate %
        %==========%
    case {1, 4, 9} %Unused flags                                                
        sys = []; 
        
        %==================%
        % Unexpected flags %
        %==================%
    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end

%
%=======================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=======================================================================
%
function  [sys,x0,str,ts] = mdlInitializeSizes(t,x,u,HWbridge_data,solver,Yecm)

ndof=size(HWbridge_data.M,1);
MP = HWbridge_data.MP;
yP = HWbridge_data.yP;
yPm = HWbridge_data.yPm;
nc=10;
nb=8;
nbm=2;
Oen=Yecm.Oen;Ocn=Yecm.Ocn;Omn=Yecm.Omn;b=Yecm.b;

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3*ndof+482+size(b,2);
sizes.NumOutputs     = length(Oen)+length(Ocn)+length(Omn);  
sizes.NumInputs      = 2+size(b,2);
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

% initialize the initial conditions

str = [];
ts  = [-1 0];

x0 = zeros(sizes.NumDiscStates,1); 
x0(3*ndof+121:3*ndof+160) = MP*ones(4*nc,1);
x0(3*ndof+161:3*ndof+200) = -MP*ones(4*nc,1);
x0(3*ndof+305:3*ndof+320) = yP*ones(2*nb,1); 
x0(3*ndof+321:3*ndof+336) = -yP*ones(2*nb,1);
x0(3*ndof+369:3*ndof+372) = yPm*ones(2*nbm,1);
x0(3*ndof+373:3*ndof+376) = -yPm*ones(2*nbm,1);
x0(3*ndof+421:3*ndof+430) = 11*ones(nc,1);
x0(3*ndof+431:3*ndof+438) = 11*ones(nb,1);
x0(3*ndof+439:3*ndof+440) = 11*ones(nbm,1);

% end mdlInitializeSizes
%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function sys = mdlUpdate(t,x,u,HWbridge_data,solver,Yecm)
CCm = HWbridge_data.C;                 % damping matrix
KKm = HWbridge_data.K;                 % initial stiffness matrix 
MMm = HWbridge_data.M;                 % mass matrix
KL = HWbridge_data.KL;                 % linear part of the global stiffness matrix
I1 = HWbridge_data.I1;                 % coefficient of ground motion
EL_bear = HWbridge_data.EL_bear;       % bearing element no.
EL_iso = HWbridge_data.EL_iso;         % bearing element no.
EL_column0 = HWbridge_data.EL_column0; % column element no.
column = HWbridge_data.column;         % plastic stiffness matrix of column element
bear = HWbridge_data.bear;             % plastic stiffness matrix of bearing element at abutmants
bearm = HWbridge_data.bearm;           % plastic stiffness matrix of bearing element at midspan

Lxy = HWbridge_data.Lxy;               % coordinate of nodes at rigid links
TRAN = HWbridge_data.TRAN;             % transfer matrix
TT_column = HWbridge_data.TT_column;   % directional cosine of bent column
elem = HWbridge_data.elem;             % structure of element stiffness and mass matrices of linear members
N_Adof = HWbridge_data.N_Adof;         % no. of active DOF
nne = HWbridge_data.nne;               % constrained DOF
nne1 = nne;
nu = HWbridge_data.nu;                 % 

MP =HWbridge_data.MP;                  % yielding moment of bent column
L = HWbridge_data.L;                   % length of column segment 
EI = HWbridge_data.EI;                 % pre-yield flexure stiffness of column
EI1 = HWbridge_data.EI1;               % post-yield flexure stiffness of column
p = HWbridge_data.p;                   % stiffness ratio       
q = HWbridge_data.q;                   
yP = HWbridge_data.yP;                 % yield displacement of bearing at abutments
yPm = HWbridge_data.yPm;               % yield displacement of bearing at midspan
kb1 = HWbridge_data.kb1;kb1m= HWbridge_data.kb1m;               % pre-yield shear stiffness of bearing
kb2 = HWbridge_data.kb2;kb2m= HWbridge_data.kb2m;               % post-yield shear stiffness of bearing

dtint = solver.dtint;                  % integration time step
alfa  = solver.alfa;                   % alfa  value for Newmark-beta Method
delta = solver.delta;                  % delta value for Newmark-beta Method
TOL   = solver.TOL;                    % specified moment tolerance of columns for nonlinear analysis
TOLb  = solver.TOLb;                   % specified displacement tolerance of bearings at bridge-ends for nonlinear analysis
TOLbm  = solver.TOLbm;                 % specified displacement tolerance of bearings at midspan for nonlinear analysis
nc =10;                                % number of nonlinear column elements
nb = 8;                                % number of nonlinear bearing elements at bridge-ends
nbm = 2;                               % number of nonlinear bearing elements at midspan
ndof = size(MMm,1);                    % number of total DOF
dt0 = dtint;                           % integration time step   
NN = elem.NN;                          % nodez of FEM model
b = Yecm.b;                            % control device location

yy=x(1:ndof);yy1=x(ndof+1:2*ndof);yy2=x(2*ndof+1:3*ndof);
key1=x(3*ndof+1:3*ndof+40);M=x(3*ndof+41:3*ndof+80);pd=x(3*ndof+81:3*ndof+120);
M_U=x(3*ndof+121:3*ndof+160);M_L=x(3*ndof+161:3*ndof+200);V=x(3*ndof+201:3*ndof+240);
Fd=x(3*ndof+241:3*ndof+256);Rd=x(3*ndof+257:3*ndof+272);Rv=x(3*ndof+273:3*ndof+288);
keyb1=x(3*ndof+289:3*ndof+304);y_U=x(3*ndof+305:3*ndof+320);y_L=x(3*ndof+321:3*ndof+336);pdb=x(3*ndof+337:3*ndof+352);
Fdm=x(3*ndof+353:3*ndof+356);Rdm=x(3*ndof+357:3*ndof+360);Rvm=x(3*ndof+361:3*ndof+364);
keybm1=x(3*ndof+365:3*ndof+368);y_Um=x(3*ndof+369:3*ndof+372);y_Lm=x(3*ndof+373:3*ndof+376);pdbm=x(3*ndof+377:3*ndof+380);
Phi=x(3*ndof+381:3*ndof+420);ikey=x(3*ndof+421:3*ndof+430);ikeyb=x(3*ndof+431:3*ndof+438);ikeybm=x(3*ndof+439:3*ndof+440);E=x(3*ndof+441:3*ndof+480);
ug10=x(3*ndof+481:3*ndof+482);ug1=ug10;
u10=x(3*ndof+482+1:3*ndof+482+size(b,2));u1=u10;  
ug2=u(1:2);u2=u(3:end);du=u2-u1;
Mm1=M;PPhi1=Phi;key10=key1;

sys=zeros(size(x));
MMm=sparse(MMm);KKm=sparse(KKm);CCm=sparse(CCm);KL=sparse(KL);
MM=zeros(4*nc,1);dMM=MM;dPhi=zeros(4*nc,1);
dRdm=zeros(2*nbm,1);dRvm=zeros(2*nbm,1);dRd=zeros(2*nb,1);dRv=zeros(2*nb,1);
kss=zeros(12,12,nc);kkbm=zeros(12,12,nbm);kkb=zeros(6,6,nb);
pdbn=zeros(2*nb,1);y_Un=yP*ones(2*nb,1);y_Ln=-yP*ones(2*nb,1);t_pbn=zeros(2*nb,1);
pdbmn=zeros(2*nbm,1);y_Umn=yPm*ones(2*nbm,1);y_Lmn=-yPm*ones(2*nbm,1);t_pbmn=zeros(2*nbm,1);
Fd1=zeros(2*nb,1);dFF=zeros(2*nb,1);tt_pnb=zeros(2*nb,1);
Fdm1=zeros(2*nbm,1);dFFm=zeros(2*nbm,1);tt_pnbm=zeros(2*nbm,1);
tt_pnc=zeros(4*nc,1);
Lexy=Lxy(1:4,:);Lwxy=Lxy(13:16,:);                          % Bearing location
edofe=[1 2 6 631 632 636];edofw=[181 182 186 637 638 642];  % Active DOFs of rigid links
TransE=TRAN([1:12],:);TransW=TRAN([1:12]+42,:);             % Transformation matrix of rigid links

dt=dt0;t2=t;
kks1=column.k_A1;                      % Elastic column stiffness 
kkb1=bear.k_A1;                        % Elastic bearing stiffness 
kkbm1=bearm.k_A1;                      % Elastic stiffness of bearings installed at top of the bent columns

if (max(ikey)==11)&(max(ikeyb)==11)&(max(ikeybm)==11)           
    kko=KKm;      
    for j=1:nc
        kss(:,:,j)=kks1;          
    end
    for j=1:nb
        kkb(:,:,j)=kkb1;
    end
    for j=1:nbm
        kkbm(:,:,j)=kkbm1;
    end    
else
    K0=KL;                             % Linear stiffness matrix excluding the stiffness matrices of bearing and column elements
    kss=zeros(12,12,nc);kkb=zeros(6,6,nb);kkbm=zeros(12,12,nbm);
    % ~~~~~~~~~~~~~~~~~~~~~~~~ %
    % --- Column stiffness --- %
    % ~~~~~~~~~~~~~~~~~~~~~~~~ %  
    for j=1:nc
        nel=EL_column0(j,1);n1=EL_column0(j,2);n2=EL_column0(j,3);
        n1dof1=NN(find(NN(:,1)==n1),5); n1dof2=NN(find(NN(:,1)==n1),6); n1dof=[n1dof1:n1dof2];
        n2dof1=NN(find(NN(:,1)==n2),5); n2dof2=NN(find(NN(:,1)==n2),6); n2dof=[n2dof1:n2dof2]; edof=[n1dof n2dof];
        [ks,kks]=IKEY_column1n(ikey(j),column,TT_column);
        %kss: global stiffness matrix of nonlinear bent column elements
        kss(:,:,j)=kks;
        K0(edof,edof)=ks+K0(edof,edof);   
    end
    
    % ~~~~~~~~~~~~~~~~~~~~~~ %
    % --- Bear stiffness --- %
    % ~~~~~~~~~~~~~~~~~~~~~~ %
    for j=1:nbm
        nel=EL_iso(j,1);n1=EL_iso(j,2);n2=EL_iso(j,3);
        n1dof1=NN(find(NN(:,1)==n1),5); n1dof2=NN(find(NN(:,1)==n1),6); n1dof=[n1dof1:n1dof2];
        n2dof1=NN(find(NN(:,1)==n2),5); n2dof2=NN(find(NN(:,1)==n2),6); n2dof=[n2dof1:n2dof2]; edof=[n1dof n2dof];
        % stiffness matrix of nonlinear bearing elements  
        kkbm(:,:,j)=IKEY_bearingm(ikeybm(j),bearm);        
        K0(edof,edof)=kkbm(:,:,j)+K0(edof,edof);   
    end
    
    for j=1:nb
        kkb(:,:,j)=IKEY_bearingn(ikeyb(j),bear);  % global stiffness matrix of nonlinear bearing elements  
    end
    
    % Account for the rigid link at east deck-end
    kbe(1,1)=kkb(1,1,1)+kkb(1,1,2)+kkb(1,1,3)+kkb(1,1,4);kbe(4,4)=kbe(1,1);kbe(1,4)=-kbe(1,1);kbe(4,1)=-kbe(1,1);
    kbe(2,2)=kkb(2,2,1)+kkb(2,2,2)+kkb(2,2,3)+kkb(2,2,4);kbe(5,5)=kbe(2,2);kbe(2,5)=-kbe(2,2);kbe(5,2)=-kbe(2,2);
    kbe(3,3)=kkb(1,1,1)*Lexy(1,2)^2+kkb(1,1,2)*Lexy(2,2)^2+kkb(1,1,3)*Lexy(3,2)^2+kkb(1,1,4)*Lexy(4,2)^2+...
        kkb(2,2,1)*Lexy(1,1)^2+kkb(2,2,2)*Lexy(2,1)^2+kkb(2,2,3)*Lexy(3,1)^2+kkb(2,2,4)*Lexy(4,1)^2;
    kbe(6,6)=kbe(3,3);kbe(3,6)=-kbe(3,3);kbe(6,3)=-kbe(3,3);
    
    % Account for the rigid link at west deck-end
    kbw(1,1)=kkb(1,1,5)+kkb(1,1,6)+kkb(1,1,7)+kkb(1,1,8);kbw(4,4)=kbw(1,1);kbw(1,4)=-kbw(1,1);kbw(4,1)=-kbw(1,1);
    kbw(2,2)=kkb(2,2,5)+kkb(2,2,6)+kkb(2,2,7)+kkb(2,2,8);kbw(5,5)=kbw(2,2);kbw(2,5)=-kbw(2,2);kbw(5,2)=-kbw(2,2);
    kbw(3,3)=kkb(1,1,5)*Lwxy(1,2)^2+kkb(1,1,6)*Lwxy(2,2)^2+kkb(1,1,7)*Lwxy(3,2)^2+kkb(1,1,8)*Lwxy(4,2)^2+...
        kkb(2,2,5)*Lwxy(1,1)^2+kkb(2,2,6)*Lwxy(2,1)^2+kkb(2,2,7)*Lwxy(3,1)^2+kkb(2,2,8)*Lwxy(4,1)^2;
    kbw(6,6)=kbw(3,3);kbw(3,6)=-kbw(3,3);kbw(6,3)=-kbw(3,3);
    K0(edofe,edofe)=kbe+K0(edofe,edofe);       K0(edofw,edofw)=kbw+K0(edofw,edofw);   
    [SK] = Boundary(K0,nu,nne1);       % boundary condition
    kko=SK;
end
m1=MMm;cc1=CCm;clear K0;
[d2,v2,a2,dy,dv]=newmark_solver(alfa,delta,m1,cc1,kko,ug1,ug2,b,u1,du,dt,yy,yy1,yy2,I1);

%Calculating the element force (M5,M6,M11,M12)
for j=1:nc/2
    zz=kss(:,:,j)*TT_column*(dy([331:342]+6*(j-1)-2));
    dMM(4*(j-1)+1:4*j)=zz([5 6 11 12]);     % 3th DOF are constrained
end
for j=1+nc/2:nc
    zz=kss(:,:,j)*TT_column*(dy([391:402]+6*(j-6)-2));
    dMM(4*(j-1)+1:4*j)=zz([5 6 11 12]);     % 183th DOF are constrained
end
MM=dMM+M;

key1n0=zeros(4*nc,1);t_p=zeros(4*nc,1);pdn=zeros(4*nc,1);M_Un=zeros(4*nc,1);M_Ln=zeros(4*nc,1);
for j=1:4*nc
    [key1n0(j),t_p(j),pdn(j),M_Un(j),M_Ln(j)]=keyec(MM(j),dMM(j),M_U(j),M_L(j),key1(j),pd(j),TOL,MP);
end       
nn=find(t_p);      
nt_pc0=length(nn);      % number of turning points (overshooting or backtracking) of column elements

%--Bearing Nonlinearity
%midspan
%33-43
zzbm=kkbm(:,:,1)*dy([185:190 431:436]);
dRdm(1:2)=dy([185:186])-dy([431:432]);
dRvm(1:2)=dv([185:186])-dv([431:432]);
dFFm(1:2)=zzbm([1 2]);
%54-44
zzbm=kkbm(:,:,2)*dy([299:304 437:442]);
dRdm(3:4)=dy([299:300])-dy([437:438]);
dRvm(3:4)=dv([299:300])-dv([437:438]);  
dFFm(3:4)=zzbm([1 2]);
FFdm=Fdm+dFFm;                         % Bearing Force vector
RRdm=Rdm+dRdm;    RRvm=Rvm+dRvm;       % Bearing deformation and velocity vectors
        
key1bmn0=zeros(2*nbm,1);t_pbm=zeros(2*nbm,1);pdbmn=zeros(2*nbm,1);y_Umn=zeros(2*nbm,1);y_Lmn=zeros(2*nbm,1);
for j=1:2*nbm
    [key1bmn0(j,1),t_pbm(j,1),pdbmn(j,1),y_Umn(j,1),y_Lmn(j,1)]=keyeb(RRdm(j),dRdm(j),y_Um(j),y_Lm(j),keybm1(j),pdbm(j),TOLbm,yPm);
end       
nnbm=find(t_pbm);        
nt_pbm0=length(nnbm);                  % number of turning points (overshooting or backtracking) of bearing elements                 

%abutment
dRd1E=TransE*dy([1 2 5]);   dRd1W=TransW*dy([180 181 184]);
dRd2E=TransE*dy([425:427]); dRd2W=TransW*dy([428:430]);
dRv1E=TransE*dv([1 2 5]);   dRv1W=TransW*dv([180 181 184]);
dRv2E=TransE*dv([425:427]); dRv2W=TransW*dv([428:430]);
for j=1:nb/2
    zzb=kkb(:,:,j)*[dRd1E(3*j-2:3*j); dRd2E(3*j-2:3*j)];
    dRd(2*j-1:2*j)=dRd1E(3*j-2:3*j-1)-dRd2E(3*j-2:3*j-1);dRv(2*j-1:2*j)=dRv1E(3*j-2:3*j-1)-dRv2E(3*j-2:3*j-1);
    dFF(2*j-1:2*j)=zzb([1 2]);
end

for j=1+nb/2:nb
    zzb=kkb(:,:,j)*[dRd1W(3*(j-nb/2)-2:3*(j-nb/2)); dRd2W(3*(j-nb/2)-2:3*(j-nb/2))];
    dRd(2*j-1:2*j)=dRd1W(3*(j-nb/2)-2:3*(j-nb/2)-1)-dRd2W(3*(j-nb/2)-2:3*(j-nb/2)-1);
    dRv(2*j-1:2*j)=dRv1W(3*(j-nb/2)-2:3*(j-nb/2)-1)-dRv2W(3*(j-nb/2)-2:3*(j-nb/2)-1);
    dFF(2*j-1:2*j)=zzb([1 2]);
end
FFd=Fd+dFF;                            % Bearing Force vector
RRd=Rd+dRd;    RRv=Rv+dRv;             % Bearing deformation and velocity vectors

key1bn0=zeros(2*nb,1);t_pb=zeros(2*nb,1);pdbn=zeros(2*nb,1);y_Un=zeros(2*nb,1);y_Ln=zeros(2*nb,1);
for j=1:2*nb
    [key1bn0(j,1),t_pb(j,1),pdbn(j,1),y_Un(j,1),y_Ln(j,1)]=keyeb(RRd(j),dRd(j),y_U(j),y_L(j),keyb1(j),pdb(j),TOLb,yP);
end       
nnb=find(t_pb);        
nt_pb0=length(nnb);                    % number of turning points (overshooting or backtracking) of bearing elements                 
nt_p0=nt_pc0+nt_pb0+nt_pbm0;           % total no. of turning point

sys(3*ndof+481:3*ndof+482)=ug2;  sys(3*ndof+482+1:3*ndof+482+size(b,2))=u2;     

if (nt_p0)==0;
    % No stiffness change    
    for j=1:nc
        ikey1(j)=IKEYc_val(key1n0([(j-1)*4+1,(j-1)*4+3]),key1n0([(j-1)*4+2,(j-1)*4+4]));
    end 
    for j=1:nbm
        ikeybm1(j)=IKEYb_val(key1bmn0(2*j-1),key1bmn0(j*2));
    end    
    for j=1:nb
        ikeyb1(j)=IKEYb_val(key1bn0(2*j-1),key1bn0(j*2));
    end    
    
    V1=zeros(4*nc,1);Phi1=zeros(4*nc,1);dPhi=zeros(4*nc,1);
    for j=1:nc
        V1(4*(j-1)+1)=(MM(4*(j-1)+2)+MM(4*(j-1)+4))/L;V1(4*(j-1)+3)=-V1(4*(j-1)+1);
        V1(4*(j-1)+2)=-(MM(4*(j-1)+1)+MM(4*(j-1)+3))/L;V1(4*(j-1)+4)=-V1(4*(j-1)+2);
    end
    
    for j=1:4*nc
        if key1(j)==0
            dPhi(j)=dMM(j)/EI;
        else
            dPhi(j)=dMM(j)/EI1;
        end
    end     
    Phi1 = Phi+dPhi;                   % Curvature of bent columns
    sys(1:ndof)=d2;sys(ndof+1:2*ndof)=v2;sys(2*ndof+1:3*ndof)=a2;
    sys(3*ndof+1:3*ndof+40)=key1n0;sys(3*ndof+41:3*ndof+80)=MM;sys(3*ndof+81:3*ndof+120)=pdn;
    sys(3*ndof+121:3*ndof+160)=M_Un;sys(3*ndof+161:3*ndof+200)=M_Ln;sys(3*ndof+201:3*ndof+240)=V1;
    sys(3*ndof+241:3*ndof+256)=FFd;sys(3*ndof+257:3*ndof+272)=RRd;sys(3*ndof+273:3*ndof+288)=RRv; 
    sys(3*ndof+289:3*ndof+304)=key1bn0;sys(3*ndof+305:3*ndof+320)=y_Un;sys(3*ndof+321:3*ndof+336)=y_Ln;sys(3*ndof+337:3*ndof+352)=pdbn;
    sys(3*ndof+353:3*ndof+356)=FFdm;sys(3*ndof+357:3*ndof+360)=RRdm;sys(3*ndof+361:3*ndof+364)=RRvm;
    sys(3*ndof+365:3*ndof+368)=key1bmn0;sys(3*ndof+369:3*ndof+372)=y_Umn;sys(3*ndof+373:3*ndof+376)=y_Lmn;sys(3*ndof+377:3*ndof+380)=pdbmn;
    sys(3*ndof+381:3*ndof+420)=Phi1;sys(3*ndof+421:3*ndof+430)=ikey1;sys(3*ndof+431:3*ndof+438)=ikeyb1;sys(3*ndof+439:3*ndof+440)=ikeybm1;
else               
    %stiffness change during an integration time step    
    jj=1; nt_pc1(jj)=nt_pc0; nt_pb1(jj)=nt_pb0; nt_pbm1(jj)=nt_pbm0;
    nt_p1(jj)=nt_pc1(jj)+nt_pb1(jj)+nt_pbm1(jj); nn1=nn;nnb1=nnb;nnbm1=nnbm;        
    t1=t2-dt0; ug1=ug10;u1=u10;   
    dt1=dt/2;ti=t1+dt1; 
    d1=yy;v1=yy1;a1=yy2;                
    M1=M;Fd1=Fd;Rd1=Rd;Rv1=Rv;
    Fdm1=Fdm;Rdm1=Rdm;Rvm1=Rvm;
    key1n1=key1; key1bn1=keyb1;key1bmn1=keybm1;         
    pdn1=pd; pdbn1=pdb;pdbmn1=pdbm;
    M_Un1=M_U; M_Ln1=M_L;Phi1=Phi;
    y_Un1=y_U; y_Ln1=y_L;
    y_Umn1=y_Um; y_Lmn1=y_Lm;
    ttn=t1*ones(2*(2*nc+nb),1);ttnc=t1*ones(4*nc,1);ttnb=t1*ones(2*nb,1);ttnbm=t1*ones(2*nbm,1);
    while (nt_p1(jj)>0)
        ugi(1,:)=interp1([t2-dt0,t2],[ug10(1) ug2(1)],ti); %interpolates to find excitation at ti
        ugi(2,:)=interp1([t2-dt0,t2],[ug10(2) ug2(2)],ti);  
        
        for j=1:size(b,2)
            ui(j,:)=interp1([t2-dt0,t2],[u10(j) u2(j)],ti);%interpolates to find force at ti
        end
        du1=ui-u1;       
        dti=ti-t1;      
        [d2,v2,a2,dy,dv]=newmark_solver(alfa,delta,m1,cc1,kko,ug1,ugi,b,u1,du1,dti,d1,v1,a1,I1);
        
        MM=zeros(4*nc,1); dMM=zeros(4*nc,1);       
        for j=1:5
            zz=kss(:,:,j)*TT_column*(dy([331:342]+6*(j-1)-2));
            dMM(4*(j-1)+1:4*j)=zz([5 6 11 12]);     % 3th, 183th are constrained
        end
        for j=6:10
            zz=kss(:,:,j)*TT_column*(dy([391:402]+6*(j-6)-2));
            dMM(4*(j-1)+1:4*j)=zz([5 6 11 12]);     % 3th, 183th are constrained
        end
        MM=dMM+M1;
        
        for j=1:nt_pc1(jj)
            [tt_pnc(nn1(j)),ttnc(nn1(j))]=Turningpointc(key1n1(nn1(j)),MM(nn1(j)),dMM(nn1(j)),M_Un1(nn1(j)),M_Ln1(nn1(j)),dt1,ti,TOL);
        end
        nntc=find(tt_pnc);nt_pc2=length(nntc);
        
        %midspan
        %33-43
        zzbm=kkbm(:,:,1)*dy([185:190 431:436]);
        dRdm(1:2)=dy([185:186])-dy([431:432]);
        dRvm(1:2)=dv([185:186])-dv([431:432]);
        dFFm(1:2)=zzbm([1 2]);
        %54-44
        zzbm=kkbm(:,:,2)*dy([299:304 437:442]);
        dRdm(3:4)=dy([299:300])-dy([437:438]);
        dRvm(3:4)=dv([299:300])-dv([437:438]);
        dFFm(3:4)=zzbm([1 2]);
        RRdm=Rdm1+dRdm;    RRvm=Rvm1+dRvm;            
                
        for j=1:nt_pbm1(jj)
            [tt_pnbm(nnbm1(j)),ttnbm(nnbm1(j))]=Turningpointb(key1bmn1(nnbm1(j)),RRdm(nnbm1(j)),dRd(nnbm1(j)),y_Umn1(nnbm1(j)),y_Lmn1(nnbm1(j)),dt1,ti,TOLbm);
        end
        nntbm=find(tt_pnbm);nt_pbm2=length(nntbm);
        
        dRd1E=TransE*dy([1 2 5]);   dRd1W=TransW*dy([180 181 184]);
        dRd2E=TransE*dy([425:427]); dRd2W=TransW*dy([428:430]);
        dRv1E=TransE*dv([1 2 5]);   dRv1W=TransW*dv([180 181 184]);
        dRv2E=TransE*dv([425:427]); dRv2W=TransW*dv([428:430]);
        for j=1:nb/2
            zzb=kkb(:,:,j)*[dRd1E(3*j-2:3*j); dRd2E(3*j-2:3*j)];
            dRd(2*j-1:2*j)=dRd1E(3*j-2:3*j-1)-dRd2E(3*j-2:3*j-1);
            dRv(2*j-1:2*j)=dRv1E(3*j-2:3*j-1)-dRv2E(3*j-2:3*j-1);
            dFF(2*j-1:2*j)=zzb([1 2]);
        end
        for j=1+nb/2:nb
            zzb=kkb(:,:,j)*[dRd1W(3*(j-nb/2)-2:3*(j-nb/2)); dRd2W(3*(j-nb/2)-2:3*(j-nb/2))];
            dRd(2*j-1:2*j)=dRd1W(3*(j-nb/2)-2:3*(j-nb/2)-1)-dRd2W(3*(j-nb/2)-2:3*(j-nb/2)-1);
            dRv(2*j-1:2*j)=dRv1W(3*(j-nb/2)-2:3*(j-nb/2)-1)-dRv2W(3*(j-nb/2)-2:3*(j-nb/2)-1);
            dFF(2*j-1:2*j)=zzb([1 2]);
        end
        FFd=Fd1+dFF;                   % Bearing force vector
        RRd=Rd1+dRd;    RRv=Rv1+dRv;   % Bearing deformation and velocity vectors
        
        for j=1:nt_pb1(jj)
            [tt_pnb(nnb1(j)),ttnb(nnb1(j))]=Turningpointb(key1bn1(nnb1(j)),RRd(nnb1(j)),dRd(nnb1(j)),y_Un1(nnb1(j)),y_Ln1(nnb1(j)),dt1,ti,TOLb);
        end
        nntb=find(tt_pnb);nt_pb2=length(nntb);
        
        ttn=[ttnc; ttnbm; ttnb];
        nt_p2=nt_pc2+nt_pbm2+nt_pb2;
        
        if max(ttn)>t2, error('error!');break; end                       
        if (min([ttnc(nn1); ttnbm(nnbm1); ttnb(nnb1)])<ti)&(min([ttnc(nn1); ttnbm(nnbm1); ttnb(nnb1)])>t1),
            nt_p2=nt_p1(jj); 
        end               
        
        if nt_p2==nt_p1(jj)
            dt1=dt1/2; ti=min([ttnc(nn1); ttnbm(nnbm1); ttnb(nnb1)]);
        else
            tt_pnc=zeros(4*nc,1);tt_pnbm=zeros(2*nbm,1);tt_pnb=zeros(2*nb,1);  
            tt_nc=zeros(4*nc,1); tt_nbm=zeros(2*nbm,1); tt_nb=zeros(2*nb,1);            
            key1nn=zeros(4*nc,1);t_pn=zeros(4*nc,1);pdnn=zeros(4*nc,1);M_Unn=zeros(4*nc,1);M_Lnn=zeros(4*nc,1);
            key1bmnn=zeros(2*nbm,1);t_pbmn=zeros(2*nbm,1);pdbmnn=zeros(2*nbm,1);y_Umnn=zeros(2*nbm,1);y_Lmnn=zeros(2*nbm,1);
            key1bnn=zeros(2*nb,1);t_pbn=zeros(2*nb,1);pdbnn=zeros(2*nb,1);y_Unn=zeros(2*nb,1);y_Lnn=zeros(2*nb,1);
            Phi1n=zeros(4*nc,1);
            for j=1:4*nc
                if key1n1(j)==0
                    dPhi(j)=dMM(j)/EI;
                else
                    dPhi(j)=dMM(j)/EI1;
                end
            end     
            Phi1n = Phi1+dPhi;
            
            for j=1:4*nc
                [key1nn(j),t_pn(j),pdnn(j),M_Unn(j),M_Lnn(j)]=keyec(MM(j),dMM(j),M_Un1(j),M_Ln1(j),key1n1(j),pdn1(j),TOL,MP);
            end  
            for j=1:2*nbm
                [key1bmnn(j,1),t_pbmn(j,1),pdbmnn(j,1),y_Umnn(j,1),y_Lmnn(j,1)]=keyeb(RRdm(j),dRdm(j),y_Umn1(j),y_Lmn1(j),key1bmn1(j),pdbmn1(j),TOLbm,yPm);
            end       
            for j=1:2*nb
                [key1bnn(j,1),t_pbn(j,1),pdbnn(j,1),y_Unn(j,1),y_Lnn(j,1)]=keyeb(RRd(j),dRd(j),y_Un1(j),y_Ln1(j),key1bn1(j),pdbn1(j),TOLb,yP);
            end       
            
            clear SK; kko=zeros(size(kko));
            for j=1:nc
                [ikey1n(j)]=IKEYc_val(key1nn([(j-1)*4+1,(j-1)*4+3]),key1nn([(j-1)*4+2,(j-1)*4+4]));
            end    
            for j=1:nbm
                [ikeybm1n(j)]=IKEYb_val(key1bmnn([(j-1)*2+1]),key1bmnn([(j-1)*2+2]));
            end    
            for j=1:nb
                [ikeyb1n(j)]=IKEYb_val(key1bnn([(j-1)*2+1]),key1bnn([(j-1)*2+2]));
            end    
            
            K0=KL; kss=zeros(size(kss)); kkbm=zeros(size(kkbm));kkb=zeros(size(kkb));
            for j=1:nc
                nel=EL_column0(j,1);n1=EL_column0(j,2);n2=EL_column0(j,3);
                n1dof1=NN(find(NN(:,1)==n1),5); n1dof2=NN(find(NN(:,1)==n1),6); n1dof=[n1dof1:n1dof2];
                n2dof1=NN(find(NN(:,1)==n2),5); n2dof2=NN(find(NN(:,1)==n2),6); n2dof=[n2dof1:n2dof2]; edof=[n1dof n2dof];
                [ks,kks]=IKEY_column1n(ikey1n(j),column,TT_column);
                kss(:,:,j)=kks;
                K0(edof,edof)=ks+K0(edof,edof);   
            end
            
            for j=1:nbm
                nel=EL_iso(j,1);n1=EL_iso(j,2);n2=EL_iso(j,3);
                n1dof1=NN(find(NN(:,1)==n1),5); n1dof2=NN(find(NN(:,1)==n1),6); n1dof=[n1dof1:n1dof2];
                n2dof1=NN(find(NN(:,1)==n2),5); n2dof2=NN(find(NN(:,1)==n2),6); n2dof=[n2dof1:n2dof2]; edof=[n1dof n2dof];
                % global stiffness matrix of nonlinear bearing elements  
                kkbm(:,:,j)=IKEY_bearingm(ikeybm(j),bearm);        
                K0(edof,edof)=kkbm(:,:,j)+K0(edof,edof);   
            end
            
            for j=1:nb
                kkb(:,:,j)=IKEY_bearingn(ikeyb(j),bear);       
            end
            kbe(1,1)=kkb(1,1,1)+kkb(1,1,2)+kkb(1,1,3)+kkb(1,1,4);kbe(4,4)=kbe(1,1);kbe(1,4)=-kbe(1,1);kbe(4,1)=-kbe(1,1);
            kbe(2,2)=kkb(2,2,1)+kkb(2,2,2)+kkb(2,2,3)+kkb(2,2,4);kbe(5,5)=kbe(2,2);kbe(2,5)=-kbe(2,2);kbe(5,2)=-kbe(2,2);
            kbe(3,3)=kkb(1,1,1)*Lexy(1,2)^2+kkb(1,1,2)*Lexy(2,2)^2+kkb(1,1,3)*Lexy(3,2)^2+kkb(1,1,4)*Lexy(4,2)^2+...
                kkb(2,2,1)*Lexy(1,1)^2+kkb(2,2,2)*Lexy(2,1)^2+kkb(2,2,3)*Lexy(3,1)^2+kkb(2,2,4)*Lexy(4,1)^2;
            kbe(6,6)=kbe(3,3);kbe(3,6)=-kbe(3,3);kbe(6,3)=-kbe(3,3);
            
            kbw(1,1)=kkb(1,1,5)+kkb(1,1,6)+kkb(1,1,7)+kkb(1,1,8);kbw(4,4)=kbw(1,1);kbw(1,4)=-kbw(1,1);kbw(4,1)=-kbw(1,1);
            kbw(2,2)=kkb(2,2,5)+kkb(2,2,6)+kkb(2,2,7)+kkb(2,2,8);kbw(5,5)=kbw(2,2);kbw(2,5)=-kbw(2,2);kbw(5,2)=-kbw(2,2);
            kbw(3,3)=kkb(1,1,5)*Lwxy(1,2)^2+kkb(1,1,6)*Lwxy(2,2)^2+kkb(1,1,7)*Lwxy(3,2)^2+kkb(1,1,8)*Lwxy(4,2)^2+...
                kkb(2,2,5)*Lwxy(1,1)^2+kkb(2,2,6)*Lwxy(2,1)^2+kkb(2,2,7)*Lwxy(3,1)^2+kkb(2,2,8)*Lwxy(4,1)^2;
            kbw(6,6)=kbw(3,3);kbw(3,6)=-kbw(3,3);kbw(6,3)=-kbw(3,3);
            K0(edofe,edofe)=kbe+K0(edofe,edofe);       K0(edofw,edofw)=kbw+K0(edofw,edofw);   
            [SK] = Boundary(K0,nu,nne1);
            kko=SK;
            
            dt=t2-ti;clear K0; 
            t1=ti; ti=t2;           
            ug1(1)=interp1([t2-dt0, t2],[ug10(1) ug2(1)],t1); 
            ug1(2)=interp1([t2-dt0, t2],[ug10(2) ug2(2)],t1); 
            for j=1:size(b,2)
                u1(j)=interp1([t2-dt0,t2],[u10(j) u2(j)],t1);
            end
            Phi1=Phi1n; 
            
            d1=d2;v1=v2;a1=a2;ugi=ug2;ui=u2; M1=MM; Fd1=FFd;Rd1=RRd;Rv1=RRv;Fdm1=FFdm;Rdm1=RRdm;Rvm1=RRvm;
            key1n1=key1nn;M_Un1=M_Unn;M_Ln1=M_Lnn;pdn1=pdnn;
            key1bn1=key1bnn;y_Un1=y_Unn;y_Ln1=y_Lnn;pdbn1=pdbnn;
            key1bmn1=key1bmnn;y_Umn1=y_Umnn;y_Lmn1=y_Lmnn;pdbmn1=pdbmnn;
            dti=ti-t1;du1=ui-u1;
            [d2,v2,a2,dy,dv]=newmark_solver(alfa,delta,m1,cc1,kko,ug1,ugi,b,u1,du1,dt,d1,v1,a1,I1);    
            
            for j=1:nc/2
                zz=kss(:,:,j)*TT_column*(dy([331:342]+6*(j-1)-2));
                dMM(4*(j-1)+1:4*j)=zz([5 6 11 12]);     %3th, 183th dof are constrained
            end
            for j=nc/2+1:nc
                zz=kss(:,:,j)*TT_column*(dy([391:402]+6*(j-6)-2));
                dMM(4*(j-1)+1:4*j)=zz([5 6 11 12]);     %3th, 183th dof are constrained
            end
            MM=dMM+M1;
            
            for j=1:4*nc
                if key1nn(j)==0
                    dPhi(j)=dMM(j)/EI;
                else
                    dPhi(j)=dMM(j)/EI1;
                end
            end     
            Phi2 = Phi1+dPhi;
            
            for j=1:4*nc
                [key1n2(j),t_p2(j),pdn2(j),M_Un2(j),M_Ln2(j)]=keyec(MM(j),dMM(j),M_Unn(j),M_Lnn(j),key1nn(j),pdnn(j),TOL,MP);
            end       
            nn2=find(t_p2); nn1=nn2';  nt_pc2=length(nn1);                           %dt
            
            %midspan
            %33-43
            zzbm=kkbm(:,:,1)*dy([185:190 431:436]);
            dRdm(1:2)=dy([185:186])-dy([431:432]);
            dRvm(1:2)=dv([185:186])-dv([431:432]);
            dFFm(1:2)=zzbm([1 2]);
            %54-44
            zzbm=kkbm(:,:,2)*dy([299:304 437:442]);
            dRdm(3:4)=dy([299:300])-dy([437:438]);
            dRvm(3:4)=dv([299:300])-dv([437:438]);
            dFFm(3:4)=zzbm([1 2]);
            FFdm=Fdm1+dFFm;                               % Bearing Force vector
            RRdm=Rdm1+dRdm;    RRvm=Rvm1+dRvm;            % Bearing deformation and velocity vectors
                    
            for j=1:2*nbm
                [key1bmn2(j,1),t_pbm2(j,1),pdbmn2(j,1),y_Umn2(j,1),y_Lmn2(j,1)]=keyeb(RRdm(j),dRdm(j),y_Umnn(j),y_Lmnn(j),key1bmnn(j),pdbmnn(j),TOLbm,yPm);
            end       
            nnbm2=find(t_pbm2); nnbm1=nnbm2'; nt_pbm2=length(nnbm1);                           
            
            dRd1E=TransE*dy([1 2 5]);   dRd1W=TransW*dy([180 181 184]);
            dRd2E=TransE*dy([425:427]); dRd2W=TransW*dy([428:430]);
            dRv1E=TransE*dv([1 2 5]);   dRv1W=TransW*dv([180 181 184]);
            dRv2E=TransE*dv([425:427]); dRv2W=TransW*dv([428:430]);
            for j=1:nb/2
                zzb=kkb(:,:,j)*[dRd1E(3*j-2:3*j); dRd2E(3*j-2:3*j)];
                dRd(2*j-1:2*j)=dRd1E(3*j-2:3*j-1)-dRd2E(3*j-2:3*j-1);dRv(2*j-1:2*j)=dRv1E(3*j-2:3*j-1)-dRv2E(3*j-2:3*j-1);
                dFF(2*j-1:2*j)=zzb([1 2]);
            end
            for j=1+nb/2:nb
                zzb=kkb(:,:,j)*[dRd1W(3*(j-nb/2)-2:3*(j-nb/2)); dRd2W(3*(j-nb/2)-2:3*(j-nb/2))];
                dRd(2*j-1:2*j)=dRd1W(3*(j-nb/2)-2:3*(j-nb/2)-1)-dRd2W(3*(j-nb/2)-2:3*(j-nb/2)-1);
                dRv(2*j-1:2*j)=dRv1W(3*(j-nb/2)-2:3*(j-nb/2)-1)-dRv2W(3*(j-nb/2)-2:3*(j-nb/2)-1);
                dFF(2*j-1:2*j)=zzb([1 2]);
            end
            FFd=Fd1+dFF;                   
            RRd=Rd1+dRd;    RRv=Rv1+dRv;
            
            for j=1:2*nb
                [key1bn2(j,1),t_pb2(j,1),pdbn2(j,1),y_Un2(j,1),y_Ln2(j,1)]=keyeb(RRd(j),dRd(j),y_Unn(j),y_Lnn(j),key1bnn(j),pdbnn(j),TOLb,yP);
            end       
            nnb2=find(t_pb2); nnb1=nnb2'; nt_pb2=length(nnb1);                           
            nt_p2=nt_pc2+nt_pbm2+nt_pb2;
            jj=jj+1;nt_p1(jj)=nt_p2;nt_pc1(jj)=nt_pc2;nt_pbm1(jj)=nt_pbm2;
            nt_pb1(jj)=nt_pb2;dt1=dt/2; ti=t1+dt1;               
        end
    end
    
    for j=1:nc
        ikey2(j)=IKEYc_val(key1n2([(j-1)*4+1,(j-1)*4+3]),key1n2([(j-1)*4+2,(j-1)*4+4]));
    end    
    for j=1:nbm
        ikeybm2(j)=IKEYb_val(key1bmn2(2*j-1),key1bmn2(j*2));
    end    
    for j=1:nb
        ikeyb2(j)=IKEYb_val(key1bn2(2*j-1),key1bn2(j*2));
    end    
    
    V1=zeros(4*nc,1);   
    for j=1:nc
        V1(4*(j-1)+1)=(MM(4*(j-1)+2)+MM(4*(j-1)+4))/L;V1(4*(j-1)+3)=-V1(4*(j-1)+1);
        V1(4*(j-1)+2)=-(MM(4*(j-1)+1)+MM(4*(j-1)+3))/L;V1(4*(j-1)+4)=-V1(4*(j-1)+2);
    end
    sys(1:ndof)=d2;sys(ndof+1:2*ndof)=v2;sys(2*ndof+1:3*ndof)=a2;
    sys(3*ndof+1:3*ndof+40)=key1n2;sys(3*ndof+41:3*ndof+80)=MM;sys(3*ndof+81:3*ndof+120)=pdn2;
    sys(3*ndof+121:3*ndof+160)=M_Un2;sys(3*ndof+161:3*ndof+200)=M_Ln2;sys(3*ndof+201:3*ndof+240)=V1;
    sys(3*ndof+241:3*ndof+256)=FFd;sys(3*ndof+257:3*ndof+272)=RRd;sys(3*ndof+273:3*ndof+288)=RRv;
    sys(3*ndof+289:3*ndof+304)=key1bn2;sys(3*ndof+305:3*ndof+320)=y_Un2;sys(3*ndof+321:3*ndof+336)=y_Ln2;sys(3*ndof+337:3*ndof+352)=pdbn;
    sys(3*ndof+353:3*ndof+356)=FFdm;sys(3*ndof+357:3*ndof+360)=RRdm;sys(3*ndof+361:3*ndof+364)=RRvm;
    sys(3*ndof+365:3*ndof+368)=key1bmn2;sys(3*ndof+369:3*ndof+372)=y_Umn2;sys(3*ndof+373:3*ndof+376)=y_Lmn2;sys(3*ndof+377:3*ndof+380)=pdbmn;
    sys(3*ndof+381:3*ndof+420)=Phi2;
    sys(3*ndof+421:3*ndof+430)=ikey2;sys(3*ndof+431:3*ndof+438)=ikeyb2;sys(3*ndof+439:3*ndof+440)=ikeybm2;
end

Mm2=sys(3*ndof+41:3*ndof+80);dMm=Mm2-Mm1;
PPhi2=sys(3*ndof+381:3*ndof+420);
dPPhi=dMm*(1/EI1-1/EI);

% Dissipated energy
for j=1:4*nc
    if key10(j)==0 dE(j)=0;
    else
        dE(j)=abs(Mm1(j)*dPPhi(j))+abs(dMm(j)*dPPhi(j))/2;
    end
    E(j)=E(j)+dE(j);
end
sys(3*ndof+441:3*ndof+480)=E;
%
%end mdlUpdate
%
%=======================================================================
% mdlOutputs
% Return the output vector for the S-function
%=======================================================================
%
function sys = mdlOutputs(t,x,u,HWbridge_data,solver,Yecm)
nb=8; nbm=2;                           % number of nonlinear bearing elements
MMm = HWbridge_data.M;                 % mass matrix
I1 = HWbridge_data.I1;                 % coefficient of ground motion
TRAN = HWbridge_data.TRAN;             % Transformation matrix
ndof = size(MMm,1);                    % number of total DOF
elem = HWbridge_data.elem;             % elements of FEM model
NNe =elem.NNe;NNw =elem.NNw;           % nodes at rigid links
NN=elem.NN;                            % nodes of bridge FEM model
N_Adof = HWbridge_data.N_Adof;         % no. of active DOF
b=Yecm.b;
Yen=Yecm.Yen;
Ycn=Yecm.Ycn;
Ymn=Yecm.Ymn;

yy=x(1:ndof);yy1=x(ndof+1:2*ndof);yy2=x(2*ndof+1:3*ndof);
key1=x(3*ndof+1:3*ndof+40);M=x(3*ndof+41:3*ndof+80);pd=x(3*ndof+81:3*ndof+120);
M_U=x(3*ndof+121:3*ndof+160);M_L=x(3*ndof+161:3*ndof+200);V=x(3*ndof+201:3*ndof+240);
Fd=x(3*ndof+241:3*ndof+256);Rd=x(3*ndof+257:3*ndof+272);Rv=x(3*ndof+273:3*ndof+288);
keyb1=x(3*ndof+289:3*ndof+304);y_U=x(3*ndof+305:3*ndof+320);y_L=x(3*ndof+321:3*ndof+336);pdb=x(3*ndof+337:3*ndof+352);
Fdm=x(3*ndof+353:3*ndof+356);Rdm=x(3*ndof+357:3*ndof+360);Rvm=x(3*ndof+361:3*ndof+364);
keybm1=x(3*ndof+365:3*ndof+368);y_Um=x(3*ndof+369:3*ndof+372);y_Lm=x(3*ndof+373:3*ndof+376);pdbm=x(3*ndof+377:3*ndof+380);
Phi=x(3*ndof+381:3*ndof+420);ikey=x(3*ndof+421:3*ndof+430);ikeyb=x(3*ndof+431:3*ndof+438);ikeybm=x(3*ndof+439:3*ndof+440);E=x(3*ndof+441:3*ndof+480);
ug10=x(3*ndof+481:3*ndof+482);ug1=ug10;
u10=x(3*ndof+482+1:3*ndof+482+size(b,2));u1=u10;  
yy2g=yy2+I1*ug10;                      %absolute acceleration
TransE0=TRAN(1:36,:);  TransW0=TRAN([1:36]+42,:);

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%     Evaluation output         %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

Oen_bsf = Yecm.Oen_bsf; 
Oen_ovm = Yecm.Oen_ovm;
Oen_msd = Yecm.Oen_msd;
Oen_msv = Yecm.Oen_msv;
Oen_msa = Yecm.Oen_msa;
Oen_cur = Yecm.Oen_cur;
Oen_brd = Yecm.Oen_brd;
Oen_brf = Yecm.Oen_brf;
Oen_eng = Yecm.Oen_eng;


sys(Oen_bsf)  = V([19 20 39 40]);             % base shear forces of bent columns in global x- and y- directions
sys(Oen_ovm)  = M([19 20 39 40]);             % overturning moments of bent columns in x- and y- directions
sys(Oen_msd)  = yy([90 91]);                  % midspan displacements in x- and y- directions
sys(Oen_msv)  = yy1([90 91]);                 % midspan velocities in x- and y- directions
sys(Oen_msa)  = yy2g([90 91]);                % midspan accelerations in x- and y- directions
sys(Oen_cur)  = Phi;                          % curvature of column
sys(Oen_brd)  = [Rd;Rdm];                     % bearing deformation output
sys(Oen_brf)  = [Fd;Fdm];                     % bearing forces 
sys(Oen_eng)  = E;                            % dissipated energy

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%     Connection output         %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
Icd=find(Ycn(:,5)==1); Ycnd=Ycn(Icd,:);         %Displacement  
Icv=find(Ycn(:,5)==2); Ycnv=Ycn(Icv,:);         %Velocity    
Ica=find(Ycn(:,5)==3); Ycna=Ycn(Ica,:);         %Acceleration
Ocd=Yen(end,1); 
Ocv=Yen(end,1)+length(Icd);
Oca=Ocv+length(Icv);
% disp. and  vel. at connection positions of devices in SIMULINK model
jn1=[];jn2=[];jjn1=[];jjn2=[];

In1d=find(Ycnd(:,2)>100); Ycnd1=Ycnd(In1d,:);%In1d=find(Ycnd1(:,5)==1);In1v=find(Ycnd1(:,5)==2);
In1v=find(Ycnv(:,2)>100); Ycnv1=Ycnv(In1v,:);
In1a=find(Ycna(:,2)>100); Ycna1=Ycna(In1a,:);

In2d=find(Ycnd(:,2)<100); Ycnd2=Ycnd(In2d,:); %In2d=find(Ycnd2(:,5)==1);In2v=find(Ycnd2(:,5)==2);
In2v=find(Ycnv(:,2)<100); Ycnv2=Ycnv(In2v,:);
In2a=find(Ycna(:,2)<100); Ycna2=Ycna(In2a,:);

for j=1:2:length(In1d)/2
    jn1 = [jn1; Ycnd1(j,2)]; jn2 = [jn2; Ycnd1(j+length(In1d)/2,2)];
    jjn1 = [jjn1; Ycnd1(j,3)]; jjn2 = [jjn2; Ycnd1(j+length(In1d)/2,3)];
end
for j=1:length(jn1)
    j1(j)= find(NNe==jn1(j));j2(j)= find(NNw==jn2(j));
    jj1(j)= find(NNe==jjn1(j));jj2(j)= find(NNw==jjn2(j));
end

for j=1:length(j1)
    sys(Ocd+2*j-1:Ocd+2*j)=TransE0(3*j1(j)-2:3*j1(j)-1,:)*yy([1 2 5])-TransE0(3*jj1(j)-2:3*jj1(j)-1,:)*yy([425 426 427]);
    sys(Ocv+2*j-1:Ocv+2*j)=TransE0(3*j1(j)-2:3*j1(j)-1,:)*yy1([1 2 5])-TransE0(3*jj1(j)-2:3*jj1(j)-1,:)*yy1([425 426 427]);
end

for j=length(j1)+1:length(j1)*2
    sys(Ocd+2*j-1:Ocd+2*j)=TransW0(3*j2(j-nb/2)-2:3*j2(j-nb/2)-1,:)*yy([180 181 184])-TransW0(3*jj2(j-nb/2)-2:3*jj2(j-nb/2)-1,:)*yy([428:430]);
    sys(Ocv+2*j-1:Ocv+2*j)=TransW0(3*j2(j-nb/2)-2:3*j2(j-nb/2)-1,:)*yy1([180 181 184])-TransW0(3*jj2(j-nb/2)-2:3*jj2(j-nb/2)-1,:)*yy1([428:430]);
end
for j=length(j1)*2+1:length(j1)*2+length(In2d)/2
    n1=Ycnd2(2*(j-length(j1)*2)-1,2);     n1dof1=NN(find(NN(:,1)==n1),5);    n1dof=find(N_Adof==n1dof1)+[0:1];
    n2=Ycnd2(2*(j-length(j1)*2)-1,3);     n2dof1=NN(find(NN(:,1)==n2),5);    n2dof=find(N_Adof==n2dof1)+[0:1];
    sys(Ocd+2*j-1:Ocd+2*j)=yy(n1dof)-yy(n2dof);
    sys(Ocv+2*j-1:Ocv+2*j)=yy1(n1dof)-yy1(n2dof);    
end

%Acceleration
Oca1=Oca;
for j=1:length(In1a)/8
    sys(Oca1+4*(j-1)+[1:2])=TransE0(3*j1(j)-2:3*j1(j)-1,:)*yy2g([1 2 5]);    
    sys(Oca1+4*(j-1)+[3:4])=TransE0(3*jj1(j)-2:3*jj1(j)-1,:)*yy2g([425 426 427]);    
end
for j=1+length(In1a)/8:length(In1a)/4
    sys(Oca1+4*(j-1)+[1:2])=TransW0(3*j2(j-nb/2)-2:3*j2(j-nb/2)-1,:)*yy2g([180 181 184]);    
    sys(Oca1+4*(j-1)+[3:4])=TransW0(3*jj2(j-nb/2)-2:3*jj2(j-nb/2)-1,:)*yy2g([428:430]);  
end

Oca2=Oca1+length(In1a);
for j=1:length(In2a)
    N_lab=Ycna2(j,2); 
    edof=Ycna2(j,4);
    n1=NN(find(NN(:,1)==N_lab),5)+edof-1;
    Cn1=find(N_Adof==n1);
    sys(Oca2+j)=yy2g(Cn1);
end

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%      Measurement output       %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

Imd=find(Ymn(:,5)==1); Ymnd=Ymn(Imd,:);         %Displacement  
Imv=find(Ymn(:,5)==2); Ymnv=Ymn(Imv,:);         %Velocity    
Ima=find(Ymn(:,5)==3); Ymna=Ymn(Ima,:);         %Acceleration

%Displacement

Imd1=[];Imd2=Imd;
for j=2:size(Imd,1)
    if Ymnd(j,1) == Ymnd(j-1,1) & Ymnd(j,4) == -Ymnd(j-1,4)
        Imd1=[Imd1;j-1;j];
    end
end
nj=[];
for j=1:length(Imd1)
    n2j=find(Imd1(j)==Imd);
    nj=[nj;n2j];
end
Imd2(nj)=[];
Ymnd1=Ymn(Imd1,:);                               %Displacement between two nodes
Ymnd2=Ymn(Imd2,:);                               %Displacement relative to ground   

Od1=length(sys);
for j=1:length(Imd1)/2
    jj=2*j-1;
    N_lab1=Ymnd1(jj,2); N_lab2=Ymnd1(jj+1,2);
    edof=Ymnd1(jj,3);
    n1=NN(find(NN(:,1)==N_lab1),5)+edof-1;
    n2=NN(find(NN(:,1)==N_lab2),5)+edof-1;
    Cn1=find(N_Adof==n1);
    Cn2=find(N_Adof==n2);
    sys(Od1+j)=yy(Cn1)*Ymnd1(jj,4)+yy(Cn2)*Ymnd1(jj+1,4);
end
Od2=Od1+length(Imd1)/2;
for j=1:length(Imd2)
    N_lab=Ymnd2(j,2); 
    edof=Ymnd2(j,3);
    n1=NN(find(NN(:,1)==N_lab),5)+edof-1;
    Cn1=find(N_Adof==n1);
    sys(Od2+j)=yy(Cn1)*Ymnd1(j,4);
end


%Velocity

Imv1=[];Imv2=Imv;
for j=2:size(Imv,1)
    if Ymnv(j,1) == Ymnv(j-1,1) & Ymnv(j,4) == -Ymnv(j-1,4)
        Imv1=[Imv1;j-1;j];
    end
end
nj=[];
for j=1:length(Imv1)
    n2j=find(Imv1(j)==Imv);
    nj=[nj;n2j];
end
Imv2(nj)=[];
Ymnv1=Ymn(Imv1,:);                               %Velocity between two nodes
Ymnv2=Ymn(Imv2,:);                               %Velocity relative to ground   

Ov1=Od2+length(Imd2);
for j=1:length(Imv1)/2
    jj=2*j-1;
    N_lab1=Ymnv1(jj,2); N_lab2=Ymnv1(jj+1,2);
    edof=Ymnv1(jj,3);
    n1=NN(find(NN(:,1)==N_lab1),5)+edof-1;
    n2=NN(find(NN(:,1)==N_lab2),5)+edof-1;
    Cn1=find(N_Adof==n1);
    Cn2=find(N_Adof==n2);
    sys(Ov1+j)=yy1(Cn1)*Ymnv1(jj,4)+yy1(Cn2)*Ymnv1(jj+1,4);
end

Ov2=Ov1+length(Imv1)/2;
for j=1:length(Imv2)
    N_lab=Ymnv2(j,2); 
    edof=Ymnv2(j,3);
    n1=NN(find(NN(:,1)==N_lab),5)+edof-1;
    Cn1=find(N_Adof==n1);
    sys(Ov2+j)=yy1(Cn1)*Ymnv1(j,4);
end

%Acceleration
Oa=Ov2+length(Imv2);
for j=1:length(Ima)
    N_lab=Ymna(j,2); 
    edof=Ymna(j,3);
    n1=NN(find(NN(:,1)==N_lab),5)+edof-1;
    Cn1=find(N_Adof==n1);
    sys(Oa+j)=yy2g(Cn1)*Ymna(j,4);
end
%
%end mdlOutputs

function [D2,V2,A2,dy,dy1,dy2]=newmark_solver(alfa,delta,Ms,Cs,Ks,ug1,ug2,b,u1,du,dt0,D1,V1,A1,I1)

%newmark_solver calculates the nonlinear structural responses using the newmark_beta method
%
%Inputs:
%    alfa, delta:   the parameter selected for the Newmark integration method. The default are, alfa=1/4;delta=1/2;
%    Ms,Cs and Ks:  the structural mass, damping and stiffness matrices
%    ug1, ug2:      the excitation at the beginning and ending moment of the calculating time increment
%    b:             a matrix denoting the control force locations
%    u1:            initial control force vector
%    du:            control force increment
%    dt0:           time step size
%    D1:            initial displacement
%    V1:            initial velocity
%    A1:            initial acceleration
%    I1:            a vector denoting the earthquake inputs

%Outputs
%    D2:            displacement over the interval dt0
%    V2:            velocity over the interval dt0
%    A2:            acceleration over the interval dt0
%    dy:            displacement increment 
%    dy1:           velocity increment 
%    dy2:           acceleration increment

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

a0=1/(alfa*dt0^2);
a1=delta/(alfa*dt0);
a2=1/(alfa*dt0);
a3=1/(2*alfa);
a4=delta/alfa;
a5=0.5*dt0*(delta/alfa-2);
a6=dt0*(1-delta);
a7=delta*dt0;
u2=u1+du;

dk=a0*Ms+a1*Cs+Ks;
dp=-Ms*I1*(ug2-ug1)+b*du+Ms*(a2*V1+a3*A1)+Cs*(a4*V1+a5*A1);
dy=dk\dp;
D2=dy+D1;
dy1=(delta/alfa/dt0)*dy-(delta/alfa)*V1+dt0*(1-0.5*delta/alfa)*A1;
V2=V1+dy1;
dy2=-I1*(ug2-ug1)+Ms\b*du-Ms\(Ks*dy+Cs*dy1); 
A2=A1+dy2;

function [ikey]=IKEYc_val(key1,key2)

%IKEYc_val   is the M-file function used to determine the element state at the current moment
%           by both end-states of a element. There are total 16 possibiities of yielding states 
%           for a bilinear biplastic beam element,which is represented by ikey. 

%Inputs:
%  key1: siffness state of M5 and M11
%        A: M5, M11 both linear
%        B: M5 linear and M11 nonlinear
%        C: M5 nonlinear and M11 linear
%        D: M5, M11 both nonlinear

%  key2: siffness state of M6 and M12
%        1: M6, M12 both linear
%        2: M6 linear and M12 nonlinear
%        3: M6 nonlinear and M12 linear
%        4: M6, M12 both nonlinear
%
%Output
%ikey:   current element state


%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

if key1(1)==0                    %M5: Linear
    if key1(2)==0                   %M11: Linear
        ikey1='A';
    else                            %M11: Nonlinear
        ikey1='B';
    end
else                             %M5: Nonlinear
    if key1(2)==0                   %M11: Linear
        ikey1='C';
    else                            %M11: Nonlinear
        ikey1='D';
    end
end

if key2(1)==0                    %M6: Linear
    if key2(2)==0                   %M12: Linear
        ikey2=1;
    else                            %M12: Nonlinear
        ikey2=2;                  
    end
else                             %M6: Nonlinear
    if key2(2)==0                   %M12: Linear
        ikey2=3;
    else                         %M12: Nonlinear
        ikey2=4;
    end
end

switch ikey1
    case 'A'
        if ikey2==1
            ikey=11;            
        elseif ikey2==2
            ikey=12;
        elseif ikey2==3
            ikey=13;
        else
            ikey=14;
        end
    case 'B'
        if ikey2==1
            ikey=21;
        elseif ikey2==2
            ikey=22;
        elseif ikey2==3
            ikey=23;
        else
            ikey=24;
        end
    case 'C'
        if ikey2==1
            ikey=31;
        elseif ikey2==2
            ikey=32;
        elseif ikey2==3
            ikey=33;
        else
            ikey=34;
        end
    otherwise
        if ikey2==1
            ikey=41;
        elseif ikey2==2
            ikey=42;
        elseif ikey2==3
            ikey=43;
        else
            ikey=44;
        end
end

function [ikeyb]=IKEYb_val(keyx,keyy)

%IKEYb_val   determines the isolator element state at the current moment/
%            There are total 4 possibilities of yielding statesfor a bilinear 
%            isolator element

%Inputs 
% keyx: 
%    A: elastic in global x dirction 
%    B: plastic in global x dirction 

% keyy: 
%    1: elastic in global y dirction 
%    2: plastic in global y dirction 

%OUTPUT
%    ikeyb:  current isolator element state

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal


if keyx==0                    %elastic  in X-direction
    ikeybx='A';
else                          %plastic  in X-direction
    ikeybx='B';
end

if keyy==0                    %elastic  in Y-direction
    ikeyby=1;
else                          %plastic  in Y-direction
    ikeyby=2;
end


switch ikeybx
    case 'A'
        if ikeyby==1
            ikeyb=11;            
        else
            ikeyb=12;
        end
    case 'B'
        if ikeyby==1
            ikeyb=21;
        else
            ikeyb=22;
        end
end


function [ks,kks]=IKEY_column1n(ikey0,column,TT_column)

%IKEY_column1n   is the function used to determine the stiffness
%                matrices of the bent column in global and local coordinate
%                systems according to the element state, ikey0

%INPUTS 
%      ikey0:     current element state
%      column:    element siffness structure
%      TT_column: directional cosine of bent column

%OUTPUTS
%      ks:     element siffness structure in global coordinate system
%      kks:    element siffness structure in local coordinate system

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal


if ikey0==11
    kks=column.k_A1;
elseif ikey0==12
    kks=column.k_A2;
elseif ikey0==13
    kks=column.k_A3;
elseif ikey0==14
    kks=column.k_A4;
elseif ikey0==21    
    kks=column.k_B1;
elseif ikey0==22
    kks=column.k_B2;
elseif ikey0==23
    kks=column.k_B3;
elseif ikey0==24
    kks=column.k_B4;
elseif ikey0==31    
    kks=column.k_C1;
elseif ikey0==32
    kks=column.k_C2;
elseif ikey0==33
    kks=column.k_C3;
elseif ikey0==34
    kks=column.k_C4;
elseif ikey0==41    
    kks=column.k_D1;
elseif ikey0==42
    kks=column.k_D2;
elseif ikey0==43
    kks=column.k_D3;
else
    kks=column.k_D4;
end

% global stiffness matrix
ks=TT_column'*kks*TT_column; 

function [kb]=IKEY_bearingn(ikeyb,bear)
%
%IKEY_bearingn   determines the stiffness matrix of a isolator element at both bridge-ends in global coordinate
%                system according to the element state, ikeyb

%INPUTS
%     ikeyb:  isolator element state 
%     bear:   isolator element stiffness structure

%OUTPUT
%      kb:    global isolator element siffness  

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

if ikeyb==11
    kb=bear.k_A1;            
elseif ikeyb==12
    kb=bear.k_A2;
elseif ikeyb==21
    kb=bear.k_B1;
else
    kb=bear.k_B2;
end

function [kbm]=IKEY_bearingm(ikeybm,bearm)
%
%IKEY_bearingm   determines the stiffness matrix of a isolator element at center bent in global coordinate
%                system according to the element state, ikeyb

%INPUTS
%     ikeybm:  midspan isolator element state   
%     bearm:   midspan isolator element stiffness structure

%OUTPUT
%     kbm:    global isolator element siffness  

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

if ikeybm==11
    kbm=bearm.k_A1;            
elseif ikeybm==12
    kbm=bearm.k_A2;
elseif ikeybm==21
    kbm=bearm.k_B1;
else
    kbm=bearm.k_B2;
end

function [keyn,t_pn,pdn,MUn,MLn]=keyec(M1,dM1,M_U,M_L,key0,pd,TOL,MP)
%
%The programme is used to check if the overshooting or backtracking occur based on 
%              the current bending moment and the last incremental bending moment of a column element-end.

%INPUTS
%        M1:     current bending moment 
%        dM1:    current incremental bending moment
%        M_U:    the upper yield bending moment at the last time interval
%        M_L:    the lower yield bending moment at the last time interval
%        key0:   the end-state at the time step intermediately preceding the current moment 
%                 key0=1:  plastic behavior in tension
%                 key0=0:  elastic behavior
%                 key0=-1: plastic behavior in compression
%        pd:     bending moment at mid-point of a loading or unloading curve
%        TOL:    torlerance defined for convergence technique
%        MP:     yield bending moment

%OUTPUTS
%        keyn:   current end state of a column element
%        t_pn:   0 (no overshooting or backtracking) 1(ovshooting or backtracking exist)
%        pdn:    updated pd
%        M_Un:   the updated upper yield bending moment at the current moment
%        M_Ln:   the updated lower yield bending moment at the current moment

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

switch key0     
    case 0      
        if M1>M_U+TOL
            keyn=1;t_pn=1;
        elseif M1<M_L-TOL
            keyn=-1;t_pn=1;
        elseif (M1>=M_L-TOL)&(M1<=M_L)
            keyn=-1;t_pn=0;
        elseif (M1>=M_U)&(M1<=M_U+TOL)
            keyn=1;t_pn=0;
        else
            keyn=0;t_pn=0;
        end            
        pdn=pd; MUn=M_U;MLn=M_L;
    case 1
        if dM1>0
            keyn=1;t_pn=0;
            pdn=pd;MUn=M_U;MLn=M_L;
        else
            keyn=0;t_pn=0;
            MUn=M1;pdn=MUn-MP;MLn=MUn-2*MP;
        end
    otherwise
        if dM1<0
            keyn=-1;t_pn=0;
            pdn=pd;MUn=M_U;MLn=M_L;
        else 
            keyn=0;t_pn=0;
            MLn=M1;pdn=MLn+MP;MUn=MLn+2*MP;
        end
end

function [keyn,t_pn,pdn,yUn,yLn]=keyeb(y,y1,y_U,y_L,key0,pd,TOLb,yP)
%
%The programme is used to check if the overshooting or backtrackingthe
%              occur based on the current deformation and relative velocity
%              of an isolator element.

%INPUTS
%       y:     current relative displacement of a isolator element
%       y1:    current relative velocity of a isolator
%       y_U:   the upper yield limit of the isolator deformation at the previous time interval
%       y_L:   the lower yield limit of the isolator deformation at the previous time interval
%       key0:  the end state of a isolator element at the at the time step intermediately preceding the current moment 
%       pd:    displacement at mid-point of a loading or unloading curve
%       TOLb:  torlerance defined for convergence technique
%       yP:    specified yield displacement

%OUTPUTS
%       keyn:   current end state
%       t_pn:   0 (no overshooting or backtracking) 1(ovshooting or backtracking exist)
%       pdn:    updated pd        
%       yUn:    the upper yield limit of the isolator deformation at the current moment
%       yLn:    the lower yield limit of the isolator deformation at the current moment

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

switch key0     
    case 0  
        if y>y_U+TOLb
            keyn=1;t_pn=1;
        elseif y<y_L-TOLb
            keyn=-1;t_pn=1;
        elseif (y>=y_L-TOLb)&(y<=y_L)
            keyn=-1;t_pn=0;
        elseif (y>=y_U)&(y<=y_U+TOLb)
            keyn=1;t_pn=0;
        else
            keyn=0;t_pn=0;
        end            
        pdn=pd; yUn=y_U;yLn=y_L;
    case 1
        if y1>0
            keyn=1;t_pn=0;
            pdn=pd;yUn=y_U;yLn=y_L;
        else
            keyn=0;t_pn=0;
            yUn=y;pdn=y-yP;yLn=y-2*yP;
        end
    otherwise
        if y1<0
            keyn=-1;t_pn=0;
            pdn=pd;yUn=y_U;yLn=y_L;
        else 
            keyn=0;t_pn=0;
            yLn=y;pdn=y+yP;yUn=y+2*yP;
        end
end

function [t_p,tn]=Turningpointc(key,M1,dM1,MU,ML,dt0,t1,TOL)

% Function Turningpointc calculates the required time increment tn of a column element, 
%          to reach the turning points at which overshooting or backtracking occur.
%
%INPUTS
%         key:    the end state at the inmidiately reached turning point
%         M1:     bending moment at the beginning moment of current time increment)
%         dM1:    current incremental bending moment
%         MU:     the upper yield limit of the end bending moment at the last time interval
%         ML:     the lower yield limit of the end bending moment at the last time interval
%         dt0:    step size
%         t1:     the ending moment of the last time increment
%         TOL:    torlerance defined for convergence technique

%OUTPUTS
%         t_p:    0 (no turning point) 1(turning point exist)
%         tn:     next time moment for turning point 

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal


if dt0<1e-6                  % minimal time increment = 1e-6
    t_p=0;
else
    switch key     
        case 0      
            if dM1>=0
                if (M1-MU<=TOL)&(M1>=MU)
                    t_p=0;
                elseif M1-MU>TOL
                    t_p=1;dt0=dt0/2;tn=t1-dt0;
                else 
                    t_p=1;dt0=dt0/2;tn=t1+dt0;
                end
                
            else
                if (M1>=ML-TOL)&(M1<=ML)
                    t_p=0;
                elseif M1-ML<-TOL
                    t_p=1;dt0=dt0/2;tn=t1-dt0;
                else 
                    t_p=1;dt0=dt0/2;tn=t1+dt0;
                end
            end
            
        case 1
            t_p=0;
            
        otherwise
            t_p=0;       
    end      
end

if t_p==0
    tn=t1;
end

function [t_p,tn]=Turningpointb(key,y,dy,y_U,y_L,dt,t1,TOLb)

%Turningpointb  determines if a turning point is reached for a specific isolator element end and 
%               calculates the time increment tn, from t1 to the turning points at which overshooting or backtracking occur.
%
%INPUTS
%         key:   the end state at the inmidiately reached turning point
%         y:     disp at t1 
%         dy:    incremental displacement
%         y_U:   the upper yield limit of the deformation of the isolator element 
%         y_L:   the lower yield limit of the deformation of the isolator element
%         t1:    the ending moment of last time increment
%         dt:    time increment
%         TOLb:  torlerance defined for convergence technique

%OUTPUTS
%         t_p:    0 (no turning point) 1(turning point exist)
%         tn:     next time moment for turning point 

%Coded by Ping TAN
%Supervised by Prof. Anil K. Agrawal

if dt<1e-6                   % minimal time increment = 1e-6
    t_p=0;
else
    switch key     
        case 0      
            if dy>=0
                if (y-y_U<=TOLb)&(y>=y_U)
                    t_p=0;
                elseif y-y_U>TOLb
                    t_p=1;dt=dt/2;tn=t1-dt;
                else 
                    t_p=1;dt=dt/2;tn=t1+dt;
                end
                
            else
                if (y>=y_L-TOLb)&(y<=y_L)
                    t_p=0;
                elseif y-y_L<-TOLb
                    t_p=1;dt=dt/2;tn=t1-dt;
                else 
                    t_p=1;dt=dt/2;tn=t1+dt;
                end
            end
            
        case 1
            t_p=0;
            
        otherwise
            t_p=0;       
    end      
end
if t_p==0
    tn=t1;
end

function [KKm] = Boundary(K,nu,nne1)

% Boundary  Modifys stiffness and mass matrices to account for boundary conditions.
%           This is done by deleting the rows and columns associated with the cons-
%           trained degrees of freedom.

% Inputs:
%       K:    the originally assembled stiffness and mass matrices
%       nu:   the constrained DOF
%       nne1: the rotaional DOFs of both abutments
% Output:
%       Km:   the modified stiffness and mass matrices

K([nu],:) = [];
K(:,[nu]) = [];

Km=K;

Km([nne1],:) = [];Km(:,[nne1]) = [];
KKm=Km;