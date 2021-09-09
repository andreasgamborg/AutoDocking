close all
clear all
clc
%%
load('Models/Primitive/otter6mtrx.mat')

U = 3;
L = zeros(6);
L(2,6) = 1;
L(3,5) = -1;
idx = [1 2 6];

CRB_lin = U*MRB*L;
CA_lin = U*MA*L;
syms u v r
D = subs(D,[u, v, r],[U, 0, 1]); % r = 1

M = MRB(idx,idx) + MA(idx,idx);
N = CRB_lin(idx,idx) + CA_lin(idx,idx) + eval(D(idx,idx));
A = -M\N;
B = inv(M);


latexeq("M_{RB}",MRB(idx,idx))
latexeq("C^*_{RB}",CRB_lin(idx,idx))
latexeq("M_{A}",MA(idx,idx))
latexeq("C^*_{A}",CA_lin(idx,idx))
latexeq("D",eval(D(idx,idx)))

latexeq("A",A);
latexeq("B",B);

S.A = A;
S.B = B;
save('Models/Primitive/otter3linmtrx.mat','-struct','S')
