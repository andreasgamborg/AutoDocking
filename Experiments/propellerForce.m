clear all
clc
%%
syms t1 t2 real
syms xi_1 xi_2 real
assume(xi_1>-pi & xi_1<pi)
assume(xi_2>-pi & xi_2<pi)

syms Tu Tv Tr
T = [Tu;Tv;Tr];

y_pont = 0.395;
r1 = [-0.8; -y_pont; 0];                           % lever arm, left propeller (m)
r2 = [-0.8; y_pont; 0];                            % lever arm, right propeller (m)

%%
eqn = [ T(1) == cos(xi_1)*t1 + cos(xi_2)*t2
    T(2) == sin(xi_1)*t1 + sin(xi_2)*t2
    T(3) == -r1(2)*cos(xi_1)*t1 + r1(1)*sin(xi_1)*t1-r2(2)*cos(xi_2)*t2 + r2(1)*sin(xi_2)*t2
    t1 == sqrt(T(1)^2 + T(2)^2)/2
    ];

%%
[t1,t2,xi_1,xi_2,parameters,conditions] = solve(eqn,[t1 t2 xi_1 xi_2],'ReturnConditions',true)



%%

T = [1 0 0]';
tau = [0 0 0.8]';
S  = Smtrx(r2)-Smtrx(r1);
 
T2 = pinv(S)*(tau - Smtrx(r1)*T)
% T1 = T - T2
T1 = pinv(S)*(Smtrx(r2)*T - tau)

T = [eye(3) eye(3); Smtrx(r1) Smtrx(r2)]*[T1; T2]





