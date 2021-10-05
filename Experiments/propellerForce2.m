syms n Va Tnn Tnv a real


eq = a == Tnn*abs(n)*n% + Tnv*n*Va;

S = solve(eq,n,'ReturnConditions', true)

O = Otter6;

Tnn = O.Propeller.Tnn;
Tnv = O.Propeller.Tnv;

a = -1


sign(a)*sqrt(abs(a)/Tnn)
