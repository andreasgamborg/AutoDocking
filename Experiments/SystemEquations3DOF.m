clear all
clc

load('Models\Primitive\SysEq6DOF_lin.mat')

%%
syms u v w p q r x y z phi theta psi    real
syms tau_u tau_v tau_w tau_p tau_q tau_r real

dnu = subs(dnu, [w p q z phi theta tau_w tau_p tau_q],zeros(1,9)); dnu = dnu([1 2 6]);
deta = subs(deta, [w p q z phi theta tau_w tau_p tau_q],zeros(1,9));deta = deta([1 2 6]);

dnu
deta
dnu_lin = jacobian(dnu, [u v r]); dnu_lin = subs(dnu_lin,[u v r],[0 0 0])


names = ["\dot{u}","\dot{v}","\dot{r}"];
for it = 1:3
    latexeq(names(it),dnu(it));
end

names = ["\dot{x}","\dot{y}","\dot{\psi}"];
for it = 1:3
    latexeq(names(it),deta(it));
end