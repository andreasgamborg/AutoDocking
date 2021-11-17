close all
clear all
clc

syms u v w p q r real
nu = [u v w p q r]';

%% Cross flow damping
load('Experiments/LinearCFdamp.mat')

%% (Linear) damping
Xu = -77.5544 ;
Yv = 0;
Zw = -546.4805;
Kp = -54.3823 ;
Mq = -246.0496;
Nr =  -45.2650;

Xh = Xu * nu(1);
Yh = Yv * nu(2);
Zh = Zw * nu(3);
Kh = Kp * nu(4);
Mh = Mq * nu(5);
Nh = Nr * (1 + 10 * abs(nu(6))) * nu(6);

tau_damp = [Xh Yh Zh Kh Mh Nh]';

%%

tau = -(tau_damp + tau_cf)

D =  jacobian(tau, nu);
D = simplify(D,100);
latexeq("\bm{D}",D)


%% Add D to model
S = load('Models/Primitive/otter6mtrx.mat');
    S.D = D;
save('Models/Primitive/otter6mtrx.mat','-struct','S');

