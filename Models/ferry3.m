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

% trim: theta = -7.5 deg corresponds to 13.5 cm less height aft maximum load
trim_setpoint = 280;

% trim_setpoint is a step input, which is filtered using the state trim_moment
persistent trim_moment;

if isempty(trim_moment)
    trim_moment = 0;
end

% Main data
g   = 9.81;         % acceleration of gravity (m/s^2)
rho = 1025;         % density of water (kg/m^3)
L = 2.0;            % length (m)
B = 1.08;           % beam (m)
m = 55.0;           % mass (kg)
rg = [0.2 0 -0.2]'; % CG for hull only (m)
R44 = 0.4 * B;      % radii of gyrations (m)
R55 = 0.25 * L;
R66 = 0.25 * L;
T_yaw = 1;          % time constant in yaw (s)
Umax = 6 * 0.5144;  % max forward speed (m/s)

% Data for one pontoon
B_pont  = 0.25;     % beam of one pontoon (m)
y_pont  = 0.395;    % distance from centerline to waterline area center (m)
Cw_pont = 0.75;     % waterline area coefficient (-)
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
nu_r = nu - [u_c v_c 0 0 0 0]';             % relative velocity vector

% Inertia dyadic, volume displacement and draft
nabla = (m+mp)/rho;                                 % water displacement volume (m^3)
T = nabla / (2 * Cb_pont * B_pont*L);               % draft
Ig_CG = m * diag([R44^2, R55^2, R66^2]);            % only hull in CG
rg = (m*rg + mp*rp)/(m+mp);                         % CG location corrected for payload
Ig = Ig_CG - m * Smtrx(rg)^2 - mp * Smtrx(rp)^2;    % hull + payload in CO

% Propeller data
l1 = [0;-y_pont;0];                     % position of port propeller (m)
l2 = [0;y_pont;0];                      % position of starboard propeller (m)
n_max =  60;                            % maximum propeller rev. (rad/s)
n_min = -60;                            % minimum propeller rev. (rad/s)

% MRB and CRB (Fossen 2021)
I3 = eye(3);
O3 = zeros(3,3);

MRB_CG = [ (m+mp) * I3  O3
    O3           Ig ];
CRB_CG = [ (m+mp) * Smtrx(nu2)         O3
    O3               -Smtrx(Ig*nu2)  ];

H = Hmtrx(rg);              % Transform MRB and CRB from the CG to the CO
MRB = H' * MRB_CG * H;
CRB = H' * CRB_CG * H;

% Hydrodynamic added mass (best practise)
Xudot = -0.1 * m;
Yvdot = -1.5 * m;
Zwdot = -1.0 * m;
Kpdot = -0.2 * Ig(1,1);
Mqdot = -0.8 * Ig(2,2);
Nrdot = -1.7 * Ig(3,3);

MA = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);
CA  = m2c(MA, nu_r);
CA(6,1) = 0; % Assume that the Munk moment in yaw can be neglected
CA(6,2) = 0; % These terms, if nonzero, must be balanced by adding nonlinear damping

% System mass and Coriolis-centripetal matrices
M = MRB + MA;
C = CRB + CA;

% Hydrostatic quantities (Fossen 2021)
Aw_pont = Cw_pont * L * B_pont;    % waterline area, one pontoon
I_T = 2 * (1/12)*L*B_pont^3 * (6*Cw_pont^3/((1+Cw_pont)*(1+2*Cw_pont)))...
    + 2 * Aw_pont * y_pont^2;
I_L = 0.8 * 2 * (1/12) * B_pont * L^3;
KB = (1/3)*(5*T/2 - 0.5*nabla/(L*B_pont) );
BM_T = I_T/nabla;       % BM values
BM_L = I_L/nabla;
KM_T = KB + BM_T;       % KM values
KM_L = KB + BM_L;
KG = T - rg(3);
GM_T = KM_T - KG;       % GM values
GM_L = KM_L - KG;

G33 = rho * g * (2 * Aw_pont);      % spring stiffness
G44 = rho * g *nabla * GM_T;
G55 = rho * g *nabla * GM_L;

G_CF = diag([0 0 G33 G44 G55 0]);   % spring stiffness matrix in the CF
LCF = -0.2;
H = Hmtrx([LCF 0 0]);               % transform G_CF from the CF to the CO
G = H' * G_CF * H;

% Natural frequencies
w3 = sqrt( G33/M(3,3) );
w4 = sqrt( G44/M(4,4) );
w5 = sqrt( G55/M(5,5) );

% Linear damping terms (hydrodynamic derivatives)
Xu = -24.4 * g / Umax;           % specified using the maximum speed
Yv = 0;
Zw = -2 * 0.3 *w3 * M(3,3);      % specified using relative damping factors
Kp = -2 * 0.2 *w4 * M(4,4);
Mq = -2 * 0.4 *w5 * M(5,5);
Nr = -M(6,6) / T_yaw;            % specified using the time constant in T_yaw

% Propeller forces and moments
n(n>n_max) = n_max;             % saturation, physical limits
n(n<n_min) = n_min;

Thrust = Propeller.K*Propeller.R*sin(Propeller.angle)*n.*abs(n) ...
    - Propeller.K*cos(Propeller.angle)*abs(n)*nu_r(1);
info.Thrust = Thrust;

Thrust = [Thrust(1) 0 ; 0 Thrust(2)];

tau1 = [cos(xi)'; sin(xi)'; zeros(1,length(xi))] * Thrust;
tau2 = Smtrx(l1)*tau1(:,1) + Smtrx(l2)*tau1(:,2);
tau = [sum(tau1,2); tau2];

% Linear damping using relative velocities + nonlinear yaw damping
Xh = Xu * nu_r(1);
Yh = Yv * nu_r(2);
Zh = Zw * nu_r(3);
Kh = Kp * nu_r(4);
Mh = Mq * nu_r(5);
Nh = Nr * (1 + 10 * abs(nu_r(6))) * nu_r(6);

tau_damp = [Xh Yh Zh Kh Mh Nh]';

% Strip theory: cross-flow drag integrals for Yh and Nh
tau_crossflow = crossFlowDrag(L,B_pont,T,nu_r);

% Ballast
g_0 = [0 0 0 0 trim_moment 0]';

% Kinematics
J = eulerang(pos(4),pos(5),pos(6));

% Time derivative of the state vector - numerical integration; see ExOtter.m
xdot = [ M \ ( tau + tau_damp + tau_crossflow - C * nu_r - G * pos - g_0)
    J * nu ];

trim_moment = trim_moment + 0.05 * (trim_setpoint - trim_moment);

end