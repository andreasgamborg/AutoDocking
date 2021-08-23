
darwAtSample = [1:1000:N, N];
modelScale = 2/9; % makes the model 2m long
propScale = 0.2;

modelx = [2,5,2,-4,-4,-2,-2,-4,-4,2,5,2,2];
modely = [1,2,3,3,1,1,-1,-1,-3,-3,-2,-1,1];
model = [modelx;modely]*modelScale;


propx = [0,-2,-3,-1,-3,-1,-2];
propy = [0,0,2,2,-2,-2,0];
prop = [propx;propy]*propScale;


propLocation = [-3 -3; 2 -2]*modelScale;


color = 'g-';
for i = darwAtSample
    R = [cos(-History.Pos(6,i)) sin(-History.Pos(6,i)); -sin(-History.Pos(6,i)) cos(-History.Pos(6,i))];
    modelR = R*model;
    
    RP = [cos(History.Propeller(1,i)) sin(History.Propeller(1,i)); -sin(History.Propeller(1,i)) cos(History.Propeller(1,i))];
    RSB = [cos(History.Propeller(2,i)) sin(History.Propeller(2,i)); -sin(History.Propeller(2,i)) cos(History.Propeller(2,i))];
    propP = R*(RP*prop+propLocation(:,1));
    propSB = R*(RSB*prop+propLocation(:,2));

    if i == N
        color = 'r-';
    end
    
    plot(modelR(1,:)+History.Pos(1,i), modelR(2,:)+History.Pos(2,i), color, 'LineWidth', 2);
    plot(propP(1,:)+History.Pos(1,i), propP(2,:)+History.Pos(2,i), color, 'LineWidth', 2);
    plot(propSB(1,:)+History.Pos(1,i), propSB(2,:)+History.Pos(2,i), color, 'LineWidth', 2);

    color = 'b-';
end


