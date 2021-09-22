close all
clear all
clc
load('Models/Primitive/otter6mtrx.mat')
%%
syms u v w p q r x y z phi theta psi    real
x = [u v w p q r x y z phi theta psi]';
nu = x(1:6);
syms tau [6 1] real

M = MRB + MA;
C = CRB + CA;
A6 = -M\(C + D);
B6 = inv(M);
dnu = A6*nu + B6*tau;
dnu = simplify(dnu);

if(0)           %<-- Print Switch
    names = ["\dot{u}","\dot{v}","\dot{w}","\dot{p}","\dot{q}","\dot{r}"];
    for it = 1:6
        latexeq(names(it),dnu(it),'multline');
    end
end
%% Find Jacobian
idx = [1 2 6];
A3 = jacobian(dnu(idx), nu(idx));
A3 = simplify(A3,1000);
B3 = jacobian(dnu(idx), tau(idx));

%% Print
if(1)
    pretty(A3)
    pretty(B3)
else
    latexeq("A",A3);
    latexeq("B",B3);
end


%% Reduction
% Linearization point
lp = [3, 0, 0, 0, 0, 0];
A = subs(A3,[u, v, w, p, q, r], lp);
B = subs(B3,[u, v, w, p, q, r], lp);

latexeq("A",A);
latexeq("B",B);

S.A = double(A);
S.B = double(B);

save('Models/Primitive/otter3mtrx.mat','-struct','S')


