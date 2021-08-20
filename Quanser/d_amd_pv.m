%% PV controller design
function [ Kp, Kv ] = d_amd_pv( Rm, Jm, Kt, Eff_m, Km, Kg, Eff_g, Mc, r_mp, Beq, PO, tp )
% I) Modelling: Laplace Transfer Function (see Maple worksheet)
% Gc(s): AMD-1 cart Open Loop Transfer Function 
% of Motor Voltage Input (in Volts) to Cart Position (in meters).
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