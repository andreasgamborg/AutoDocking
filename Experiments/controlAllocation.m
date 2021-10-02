clear all
clc
%%

y_pont = 0.395;
r1 = [-0.8; -y_pont; 0];                           % lever arm, left propeller (m)
r2 = [-0.8; y_pont; 0];                            % lever arm, right propeller (m)

B = [eye(3) eye(3); Smtrx(r1) Smtrx(r2)];
W = eye(6);
iW = inv(W);

T = [5 0 0 0 0 2]';

%% Pseudo Inverse
tau(:,1) = pinv(B)*T;
%% Damped least-squares inverse
e = 1e-6;
C = iW*B' * inv(B*iW*B'+e*eye(6));

tau(:,2) = C*T;
%% SVD
[U,S,V] = svd(B*iW*B');
S(S<eps) = 0;
iS = S;
iS(S~=0) = 1./S(S~=0);

C = iW*B'*V*iS*U';
tau(:,3) = C*T;

%%

tau1 = tau(1:3,3);
tau2 = tau(4:6,3);

xi1 = atan2(tau1(2),tau1(1));
xi2 = atan2(tau2(2),tau2(1));

a1 = norm(tau1);
a2 = norm(tau2);

disp('Angles:')
disp([xi1;xi2])
disp('Forces:')
disp([a1;a2])


%% Check
disp('Net force an moment:')
T = B*[cos(xi1) 0; sin(xi1) 0; 0 0; 0 cos(xi2); 0 sin(xi2); 0 0]*[a1; a2]




