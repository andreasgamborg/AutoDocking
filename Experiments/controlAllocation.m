clear all
clc
%%

y_pont = 0.395;
r1 = [-0.8; -y_pont; 0];                           % lever arm, left propeller (m)
r2 = [-0.8; y_pont; 0];                            % lever arm, right propeller (m)

B = [eye(3) eye(3); Smtrx(r1) Smtrx(r2)];
W = eye(6);
iW = inv(W);

T = [1 1 0 0 0 4]';

%% Pseudo Inverse
u(:,1) = pinv(B)*T;
%% Damped least-squares inverse
e = 1e-6;
C = iW*B' * inv(B*iW*B'+e*eye(6));

u(:,2) = C*T;
%% SVD
[U,S,V] = svd(B*iW*B');
S(S<eps) = 0;
iS = S;
iS(S~=0) = 1./S(S~=0);

C = iW*B'*V*iS*U';
u(:,3) = C*T;

%%

u

T = B*u

