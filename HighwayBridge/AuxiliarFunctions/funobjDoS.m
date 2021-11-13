function fobj = funobjDoS(seleccion, A, B, C, D, E, Gain)
% Determines the objective function of the DoS attack based on the H2 norm
% of the system

% Filter
filter = tf(121, [1 11/0.707 121]);

    matrix_mask = diag(1-seleccion);
    K = matrix_mask*Gain;
    Acl = A + B*K;
    Ccl = C + D*K;
    siscl = ss(Acl,E,Ccl,0);
    tranfer = tf(siscl);
    tranfer = tranfer*filter;
    
    
    fobj = -norm( tranfer(:,1)); 
