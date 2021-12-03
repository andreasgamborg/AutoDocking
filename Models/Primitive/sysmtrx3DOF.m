function [M,C,D] = sysmtrx3DOF(nu)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here

    u = nu(1);
    v = nu(2);
    r = nu(3);


    MRB = [
        55.0000       0           0
        0       55.0000     11.0000
        0       11.0000     18.1500
        ];

    MA =    [
        5.5000        0       0
        0       82.5000       0
        0             0 27.1150
        ];

    M = MRB + MA;

    CRB = [
        [    0,       -55*r,                 -11*r]
        [ 55*r,           0,                    0]
        [ 11*r,           0,                   0]
        ];


    CA = [
        [    0,    0,           -82.5000*v]
        [    0,    0,            5.5000*u]
        [    82.5000*v,   -5.5000*u,      0]
        ];

    C = CRB + CA;



    DL = [
        [77.5544,   0,               0]
        [      0,   122,             - 4.8994]
        [      0,   0,              49.3007]
        ];
    DNL = [
        [      12.5*abs(u) + 2.4*u^2,     0,        0   ]
        [       0, 2.3*abs(v)+0.7*abs(r)          3.4*abs(v)+0.8*abs(r)   ]
        [          0, 0.28*abs(v)-0.18*abs(r) ,          -0.1*abs(v)+1.7*abs(r) ]
        ];
    
    D = DL + DNL*2;

end

