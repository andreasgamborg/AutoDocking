clear all
load('Models/Primitive/otter6mtrx.mat')

syms u v w p q r x y z phi theta psi    real
x = [u v w p q r x y z phi theta psi]';
%%
if(0)           %<-- Switch
    latexeq("M_{RB}",MRB)
    latexeq("C_{RB}",CRB)
    latexeq("M_{A}",MA)
    latexeq("C_{A}",CA)
    latexeq("D",D)
end

%% System Equations
M = MRB + MA;
C = CRB + CA;

A = -M\(C + D);
B = inv(M);
B_c = M\(CA + D);

index = [1 2 6];

latexeq("A",A(index,index))
latexeq("B",B(index,index))
latexeq("B_c",B_c(index,index))

% "A\nu = "+latex(simplify(A*nu))

%% Reduction
w = 0;
p = 0;
q = 0;

%A = subs(A,[w, p, q],[0, 0, 0]);

A = eval(A);
B_c = eval(B_c);

latexeq("A",A(index,index))
latexeq("B",B(index,index))
latexeq("B_c",B_c(index,index))


S.A = A(index,index);
S.B = B(index,index);
S.B_c = B_c(index,index);
save('Models/otter3mtrx.mat','-struct','S')



