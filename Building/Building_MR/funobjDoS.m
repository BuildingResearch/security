function fobj = funobjDoS(seleccion, A, E, B, C, D, Gain)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% filtro
filtro = tf(121, [1 11/0.707 121]);

    matrix_mask = diag(1-seleccion);
    K = matrix_mask*Gain;
    Acl = A - B*K;
    Ccl = C;
    siscl = ss(Acl,E,Ccl,0);
    tranfer = tf(siscl);
    tranfer = tranfer*filtro;
    [sv,~]=sigma(tranfer);
%     fobj=-max(sv);
    
    fobj = -norm(tranfer(1:20,1),2); 
