function P = vesselplot(yaw,xi)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

modelScale = 2/9; % makes the model 2m long
propScale = 0.4;

modelx = [2,5,2,-4,-4,-2,-2,-4,-4, 2, 5, 2,2];
modely = [1,2,3, 3, 1, 1,-1,-1,-3,-3,-2,-1,1];
model = [modelx;modely]*modelScale;


propx = [0,-2,-3,-1,-3,-1,-2,0];
propy = [0,0,2,2,-2,-2,0,0];
prop = [propx;propy]*modelScale*propScale;

propLocation = [-2 -2; -2 2]*modelScale;

R = [cos(-yaw) sin(-yaw); -sin(-yaw) cos(-yaw)];
    modelR = R*model;
    
RP = [cos(-xi(1)) sin(-xi(1)); -sin(-xi(1)) cos(-xi(1))];
RSB = [cos(-xi(2)) sin(-xi(2)); -sin(-xi(2)) cos(-xi(2))];
propP = R*(RP*prop+propLocation(:,1));
propSB = R*(RSB*prop+propLocation(:,2));


P = [modelR(:,1:6) propP modelR(:,6:7) propSB modelR(:,7:end)];

end

