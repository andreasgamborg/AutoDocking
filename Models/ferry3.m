function [xdot, info] = ferry3(Input)
% [xdot,inf] = otter(Input)
%
% Input.x = [ u v r x y psi ]'
%   u:     surge velocity          (m/s)
%   v:     sway velocity           (m/s)
%   r:     yaw velocity            (rad/s)
%   x:     position in x direction (m)
%   y:     position in y direction (m)
%   psi:   yaw angle               (rad)
%
% State of the propellers speed and angle
% Input.prop. 
%   velo    = [veloP veloSB]'   Speed of Port and Starboard propeller (rad/s)
%   xi      = [xiP xiSB]'       Angle of Port and Starboard propeller (rad)
%
% Payload of the vessel
% Input.payload. 
%   m                           Mass of the payload (kg)
%   r       = [x, y, z]'        Position of the payload (m)
%
% Current
% Input.current. 
%   v                           current speed (m/s)
%   beta                        current direction (rad)


% Check dimensions of input
if (length(Input.x) ~= 6),error('x vector must have dimension 6!'); end

% Main data
g   = 9.81;         % acceleration of gravity (m/s^2)
rho = 1025;         % density of water (kg/m^3)
L = 6.0;            % length (m)
B = 2.0;            % beam (m)
m = 1000.0;         % mass (kg)
rg = [0.2 0]';    % CG for hull only (m)
Gy_yaw = 0.25 * L;     % radii of gyrations (m)

% Data for one pontoon
B_pont  = 0.25;     % beam of one pontoon (m)
y_pont  = 0.395;    % distance from centerline to waterline area center (m)
Cb_pont = 0.4;      % block coefficient, computed from m = 55 kg

% Propeller
Propeller.R = 0.03;
Propeller.C = 0.01;
Propeller.angle = pi/8;
Propeller.blades = 3;
Propeller.K = rho/2*pi*Propeller.R*Propeller.C*Propeller.blades;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State
nu = Input.x(1:3);                          % velocities
pos = Input.x(4:6);                         % positions
%U = sqrt(nu(1)^2 + nu(2)^2);                % speed

% current
u_c = Input.current.v * cos(Input.current.beta - pos(3));           % current surge velocity
v_c = Input.current.v * sin(Input.current.beta - pos(3));           % current sway velocity
nu_r = nu - [u_c v_c 0]';             % relative velocity vector

% Inertia dyadic, volume displacement and draft
nabla = (m+Input.mp)/rho;                                 % water displacement volume (m^3)
T = nabla / (2 * Cb_pont * B_pont*L);               % draft
Ig_CG = m * Gy_yaw^2;            % only hull in CG
rg = (m*rg + Input.mp*Input.rp)/(m+Input.mp);                         % CG location corrected for payload
Ig = Ig_CG + Input.mp*norm(rg-Input.rp);

% Propeller data
l1 = [-L*0.4;-y_pont];                  % position of port propeller (m)
l2 = [-L*0.4;y_pont];                   % position of starboard propeller (m)
n_max =  60;                            % maximum propeller rev. (rad/s)
n_min = -60;                            % minimum propeller rev. (rad/s)


% Rigid Body kinetics
M = [ (m+Input.mp) * eye(2)  zeros(2,1)
            zeros(1,2)           Ig ];
        
C = [ 0             0            -M(2,2)*nu(2)-M(2,3)*nu(3)
        0             0             M(1,1)*nu(1)
        M(2,2)*nu(2)+M(2,3)*nu(3)  -M(1,1)*nu(1)   0              ];
    
D = [1 0 0;
    0 1 0.1;
    0 0.1 1];
D = D*100;

% Propeller forces and moments
n = Input.prop.velo;
xi = Input.prop.xi;

n(n>n_max) = n_max;             % saturation, physical limits
n(n<n_min) = n_min;

Thrust = Propeller.K*Propeller.R*sin(Propeller.angle)*n.*abs(n) ...
    - Propeller.K*cos(Propeller.angle)*abs(n)*nu_r(1);
info.Thrust = Thrust;

Thrust = [Thrust(1) 0 ; 0 Thrust(2)];

tau1 = [cos(xi)'; sin(xi)'] * Thrust;

l1_h = [-l1(2); l1(1)];
l2_h = [-l2(2); l2(1)];

y1 = dot(tau1(:,1),l1_h) / norm(l1_h);
y2 = dot(tau1(:,2),l2_h) / norm(l2_h);

tau2 = y1 + y2;
tau = [sum(tau1,2); tau2];


% Kinematics

psi = Input.x(6);
R = [   cos(psi)   -sin(psi)    0
        sin(psi)    cos(psi)    0
        0           0           1 ];

% Time derivative of the state vector - numerical integration; see ExOtter.m
nudot = M \ ( tau - C * nu_r - D * nu_r);
etadot = R * nu ;
xdot = [nudot; etadot];
end