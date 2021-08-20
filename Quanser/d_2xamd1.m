% D_AMD2
%
% Controller design for the Active Mass Damper - Two-Floor (AMD-2) Experiment.
%
% D_AMD2 first designs a Proportional-plus-Velocity (PV) controller 
% for the AMD-2 cart system.
% D_AMD2 then designs a full-order observer as well as 
% a state-feedback controller for the AMD-2 plant.
%
% Controller nomenclature:
% Kp    Proportional Gain (V/m)
% Kv    Velocity Gain (V.s/m)
% K     State-Feedback Vector
% G     Full-Order Observer Gain Matrix
%
% Copyright (C) 2003 Quanser Consulting Inc.
% Quanser Consulting Inc.


function [ Kp, Kv, K, G ] = d_2xamd1( Rm, Jm, Kt, Km, Kg, Mc, r_mp, Beq, PO, tp, A, B, C, D, OP, Q, R, X0 )
% PV controller design
[ Kp, Kv ] = d_amd_pv( Rm, Jm, Kt, Km, Kg, Mc, r_mp, Beq, PO, tp );
% Full-order observer and state-feedback controller designs
[ K, G ] = d_amd2_obs_sf( A, B, C, D, OP, Q, R, X0 );
% end of function 'd_amd2'


%% PV controller design
function [ Kp, Kv ] = d_amd_pv( Rm, Jm, Kt, Km, Kg, Mc, r_mp, Beq, PO, tp )
% I) Modelling: Laplace Transfer Function (see Maple worksheet)
% Gc(s): AMD-2 cart Open Loop Transfer Function 
% of Motor Voltage Input (in Volts) to Cart Position (in meters).
Eff_m = 1;
Eff_g = 1;
Gc_NUM(1) = Eff_g * Kg * Eff_m * Kt * r_mp;
Gc_DEN(1) = Mc * Rm * r_mp^2 + Rm * Eff_g * Kg^2 * Jm;
Gc_DEN(2) = Beq * Rm * r_mp^2 + Eff_g * Kg^2 * Eff_m * Kt * Km;
Gc_DEN(3) = 0;
% open-loop system
Gc_SYS = tf( Gc_NUM, Gc_DEN );

% II) calculate the required controller gains, Kp and Kv
% meeting the desired specifications
% i) spec #1: maximum Percent Overshoot (PO)
if ( PO > 0 )
    % using the *Hint provided in the lab, zeta_c_min is given by:
    zeta_c_min = abs( log( PO / 100 ) ) / sqrt( pi^2 + log( PO / 100)^2 );
    zeta_c = zeta_c_min;
else
    error( 'Error: Set Percentage Overshoot.' )
end
% ii) spec #2: tp - using the *Hint provided in the lab:
wn_c = pi / ( tp * sqrt( 1 - zeta_c^2 ) );
% alternative analytical form for wn_c:
%wn_c = sqrt( log( PO / 100 )^2 + pi^2 ) / tp;

% Using the control law: Vm = Kp * ( xc_d - xc ) - Kv * xc_dot
% PV controller gain settings (see Maple worksheet):
Kp = ( wn_c^2 * Gc_DEN(1) - Gc_DEN(3) ) / Gc_NUM(1);
Kv = ( 2 * zeta_c * wn_c * Gc_DEN(1) - Gc_DEN(2) ) / Gc_NUM(1);
% alternatively, analytical form (see Maple worksheet):
%Kp = wn_c^2 * Rm * ( Mc * r_mp^2 + Eff_g * Kg^2 * Jm ) / ( r_mp * Eff_g * Kg * Eff_m * Kt );
%Kv = ( 2 * zeta_c * wn_c * Rm * Mc * r_mp^2 + 2 * zeta_c * wn_c * Rm * Eff_g * Kg^2 * Jm - Eff_g * Kg^2 * Eff_m * Kt * Km - Beq * Rm * r_mp^2 ) / ( r_mp * Eff_g * Kg * Eff_m * Kt );

%% optional: 
% flag to carry out some system analysis
%SYS_PV_ANALYSIS = 'YES';
SYS_PV_ANALYSIS = 'NO';
if strcmp( SYS_PV_ANALYSIS, 'YES' )
    % III) Closed-Loop System Response Simulation
    % carrying out block diagram reduction:
    % i) first of the inner velocity loop
    KV_SYS = tf( [ Kv 0 ], [ 1 ] );
    CART_KV_OL_SYS = feedback( Gc_SYS, KV_SYS );
    % ii) then of the outer position loop
    % to obtain the Overall Closed-Loop Transfer Function of the feedback-controlled system
    Kp_CART_KV_OL_SYS = Kp * CART_KV_OL_SYS;
    Kp_CART_KV_CL_SYS = feedback( Kp_CART_KV_OL_SYS, 1 );
    % CL poles, damping, and natural frequency
    damp( Kp_CART_KV_CL_SYS )
    % Simulated Unit Step Response of the Closed Loop System (without UPM saturation)
    figure ( 1 )
    step( Kp_CART_KV_CL_SYS )
    set( 1, 'name', strcat( 'AMD-2 cart with PV Controller: Kp= ', num2str( Kp ), ' V/m, Kv= ', num2str( Kv ), ' V.s/m') )
    grid on
end
% end of function 'd_amd_pv( )'


%% Full-order observer and state-feedback controller designs
function [ K, G ] = d_amd2_obs_sf( A, B, C, D, OP, Q, R, X0 )
% state vector: X = [ xc1; xc2; xf1; xf2; xc1_dot; xc2_dot; xf1_dot; xf2_dot ]; 
% input: U = [Vm]
% output vector: Y = [ xc; xf1_ddot; xf2_ddot ]

%
% State-feedback controller design: calculate K
% state-feedback gain vector calculation: LQR
K = lqr(A,B,Q,R);
%
% Full-Order Observer design: calculate G
% observer gain matrix calculation: pole placement
G = place( A', C', OP )';


%% optional: 
% flag to carry out some additional system analysis
%SYS_OBS_ANALYSIS = 'YES';
SYS_OBS_ANALYSIS = 'NO';
if strcmp( SYS_OBS_ANALYSIS, 'YES' )
    % open-loop pole-zero structure:
    %eig( A )
    disp( 'open-loop system: ' )
    [ z_ol, p_ol, k_ol ] = ss2zp( A, B, C, D, 1 )
    % AMD-2 open-loop system
    AMD2_OL_SYS = ss( A, B, C, D );
    % Closed-Loop poles, damping, and natural frequency
    damp( AMD2_OL_SYS )
    
    % i) observer design possible iff (A,C) is observable
    % observability matrix: [ C; CA; CA^2; CA^3; CA^4; CA^5 ]
    Wo = obsv( A, C )
    % the system is observable iff Wo has full rank (i.e. = number of states)
    % number of uncontrollable states
    unob_obs = length( A ) - rank( Wo )    % = 0
    % ii) pole placement is possible iff (A',C') is controllable
    % controllability matrix: [ C', A'.C', A'^2.C', A'^3.C', A'^4.C', A'^5.C' ]
    Wc = ctrb( A', C' );
    % the system is controllable iff Wc has full rank (i.e. = number of states)
    % number of uncontrollable states
    unco_obs = length( A' ) - rank( Wc );    % = 0

    % i) pole placement is possible iff (A,B) is controllable
    % controllability matrix: [ B, A.B, A^2.B, A^3.B, A^4.B, A^5.B ]
    Wc_sf = ctrb( A, B );
    % the system is controllable iff Wc_sf has full rank (i.e. = number of states)
    % number of uncontrollable states
    unco_sf = length( A ) - rank( Wc_sf );    % = 0
end
% end of function 'd_amd2_obs_sf( )'
