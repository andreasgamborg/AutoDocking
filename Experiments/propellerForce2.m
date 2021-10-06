clear all
syms n a real positive
syms Va Tnn Tnv real
assumeAlso(Tnn>0)
assumeAlso(Tnv<0)

eq1 = a == Tnn*abs(n)*n + Tnv*abs(n)*Va;

eq2 = a == Tnn*n^2 + Tnv*n*Va;
eq3 = a == Tnn*n^2;


%%
S = solve([eq2],n,'ReturnConditions', true)
S.n

%%
O = Otter6;

Tnn = O.Propeller.Tnn;
Tnv = O.Propeller.Tnv;

%%
% a = -1
% sign(a)*sqrt(abs(a)/Tnn)
a = -1;
Va = 0;


n = sign(a)*sqrt(abs(a)/Tnn)

n = sign(a)*(sqrt(Tnv^2*Va^2 + 4*Tnn*abs(a))-Tnv*Va) / (2*Tnn)


-(0.5000*Tnv*Va - 0.5000*(Tnv^2*Va^2 + 4*Tnn*a)^(1/2))/Tnn
 
-(Tnv*Va - sqrt(Tnv^2*Va^2 + 4*Tnn*a))/(2*Tnn)
