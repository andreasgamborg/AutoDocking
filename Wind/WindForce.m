function [F] = WindForce(inputArg1,inputArg2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

load('Wind\Cinter.mat','C');

rho = 1.225;        % Density of air (kg/m^3)

% dimensions
h = 0.8;            % Height    (m)
l = 2.0;            % Length    (m)
w = 1.08;           % Width     (m)


% Area
Ax = h*w;
Ay = h*l;

% Coefficients
disdir = mod(round(dir),360);
Cx = C(disdir,1);
Cy = C(disdir,2);
Ck = C(disdir,3);      
Cn = C(disdir,4); 

% Force
Fx =  0.5*rho*Cx*Ax*V^2;
Fy =  0.5*rho*Cy*Ay*V^2;
F = [Fx;Fy];

end

