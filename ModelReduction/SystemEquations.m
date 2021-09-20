close all
clear all
clc
%%
load('Models/Primitive/otter6mtrx.mat')


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
        latexeq(names(it),dnu(it));
    end
end
%% Find Jacobian
idx = [1 2 6];
A = jacobian(dnu, nu);
B = jacobian(dnu, tau);

%% Print
if(1)
    pretty(A(idx,idx))
    pretty(B(idx,idx))
else
    latexeq("A",A(idx,idx));
    latexeq("B",B(idx,idx));
end


%% Reduction
w = 0;
p = 0;
q = 0;
% Linearization point
u = 3;
v = 0;
r = 0;

%A = subs(A,[w, p, q],[0, 0, 0]);
A = eval(A);
B = eval(B);

latexeq("A",A(idx,idx));
latexeq("B",B(idx,idx));


S.A = A(idx,idx);
S.B = B(idx,idx);
save('Models/Primitive/otter3mtrx.mat','-struct','S')


