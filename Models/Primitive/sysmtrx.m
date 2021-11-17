function [M,C,DL,DNL,G] = sysmtrx(nu)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

            u = nu(1);
            v = nu(2);
            w = nu(3);
            p = nu(4);
            q = nu(5);
            r = nu(6);
            

            MRB = [
                55.0000         0         0         0  -11.0000         0
                0   55.0000         0   11.0000         0   11.0000
                0         0   55.0000         0  -11.0000         0
                0   11.0000         0   14.6643         0    4.4000
                -11.0000         0  -11.0000         0   22.5500         0
                0   11.0000         0    4.4000         0   18.1500
                ];
            
            MA =    [
                
            5.5000         0         0         0         0         0
            0   82.5000         0         0         0         0
            0         0   55.0000         0         0         0
            0         0         0    2.4929         0         0
            0         0         0         0   14.5200         0
            0         0         0         0         0   27.1150
            ];
        
        M = MRB + MA;
        
        CRB = [
            [    0,       -55*r,  55*q,                  -11*r,                  -11*q,                -11*r]
            [ 55*r,           0, -55*p,                      0,            11*p - 11*r,                    0]
            [-55*q,        55*p,     0,                   11*p,                   11*q,                 11*p]
            [ 11*r,           0, -11*p,                      0,   4.4000*p + 13.7500*r,           -18.1500*q]
            [ 11*q, 11*r - 11*p, -11*q, - 4.4000*p - 13.7500*r,                      0, 10.2643*p + 4.4000*r]
            [ 11*r,           0, -11*p,              18.1500*q, - 10.2643*p - 4.4000*r,                    0]
            ];
        
        
        CA = [
            [    0,    0,          0,          0,      55*w, -82.5000*v]
            [    0,    0,          0,      -55*w,         0,   5.5000*u]
            [    0,    0,          0,  82.5000*v, -5.5000*u,          0]
            [    0, 55*w, -82.5000*v,          0, 27.1150*r, -14.5200*q]
            [-55*w,    0,   5.5000*u, -27.1150*r,         0,   2.4929*p]
            [    0,    0,          0,  14.5200*q, -2.4929*p,          0]
            ];
        
        C = CRB + CA;
        
        G = 1.0e+03*[
            0         0         0         0         0         0
            0         0         0         0         0         0
            0         0    7.5414         0    1.5083         0
            0         0         0    1.0773         0         0
            0         0    1.5083         0    2.8534         0
            0         0         0         0         0         0
            ];
        
        
        DL = [
            [77.5544,   0,        0,       0,        0,        0]
            [      0,   0,        0,       0,        0, - 4.8994]
            [      0,   0, 546.4805,       0,        0,        0]
            [      0,   0,        0, 54.3823,        0,        0]
            [      0,   0,        0,       0, 246.0496,        0]
            [      0,   0,        0,       0,        0,  49.3007]
            ];
        DNL = [
            [      0,                                                                     0,        0,       0,        0,                                                       0]
            [      0, 1.7764e-15*sign(v)*(7.4603e+15*r + 1.3043e+17*v + 2.8797e+15*sign(v)),        0,       0,        0,   35.9154*abs(r) + 13.2522*abs(v) + 35.9154*r*sign(r)  ]
            [      0,                                                                     0,        0,       0,        0,                                                       0]
            [      0,                                                                     0,        0,       0,        0,                                                       0]
            [      0,                                                                     0,        0,       0,        0,                                                       0]
            [      0, 8.8818e-16*sign(v)*(8.8677e+16*r + 1.1887e+16*v - 4.6509e+15*sign(v)),        0,       0,        0,   460.1313*abs(r) + 78.7613*abs(v) + 460.1313*r*sign(r)]
            ];
        
end

