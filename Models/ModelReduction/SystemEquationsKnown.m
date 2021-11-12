close all
clear all
clc
load('Models/Primitive/otter6mtrx.mat')
load('Experiments/tau_damp.mat')
load('CrossFlowDragFit/tau_cf11.mat')
tau_cf = tau_cf11;
%%
syms u v w p q r x y z phi theta psi    real
nu = [u v w p q r]';
eta = [x y z phi theta psi]';
tau = sym("tau_"+["u" "v" "w" "p" "q" "r"]','real');

%%
M = MRB + MA;
C = CRB + CA;
%dnu = -M\C*nu -M\G*eta + M\(tau + tau_cf + tau_damp);
dnu = M\C*nu;

dnu = simplify(dnu,100);

J = eulerang(eta(4),eta(5),eta(6));
deta = J*nu;
deta = simplify(deta,100);


dnu = subs(dnu,[w p q], [0 0 0]);


%% Print
if(1)           %<-- Print Switch
    names = ["\dot{u}","\dot{v}","\dot{w}","\dot{p}","\dot{q}","\dot{r}"];
    for it = 1:6
        latexeq(names(it),dnu(it),'multline');
    end

    names = ["\dot{x}","\dot{y}","\dot{z}","\dot{\phi}","\dot{\theta}","\dot{\psi}"];
    for it = 1:6
        latexeq(names(it),deta(it));
    end
end

%%
S.nu = nu;
S.dnu = dnu;
S.eta = eta;
S.deta = deta;
S.tau = tau;
%save('Models/Primitive/SysEq6DOF_lin.mat','-struct','S')

