classdef Otter < handle
    %OTTER Model class of the otter ASV
    %   This class decribe the dynamics of a vessel simular to the Otter.
    %   The aim is to describe vessel dynamics as close as possible to the
    %   real-world dynamics. The Otter is af twin-hull vessel with two
    %   azimuth thrusters.

    %% State - System Matrices - Input
    properties
        State
        % Mass/Inertia
        M               % MRB + MA
        MRB
        MA
        % Coriolis
        C               % CRB + CA
        CRB
        CA
        % Damping Forces
        D               % DL + DNL
        DL
        DNL
        % Restoring Forces
        G
        %
        Thrust
    end
    %% Propellers
    properties
        UseProppeller % boolean for switching between tau or [n xi] as actuator
        Propeller       % Object for thruster dynamics
    end
    %% Distrubance & Habour
    properties
        useDistrubance  % flag
        Payload
        Current
        Wind
        Harbour         % Object for Habour dynamics see function setHarbour() for structure
    end
    %% History & Plot
    properties
        History
        t = 0;
    end
    %% Other properties
    properties
        draft
        Ig      % Inertia matrix corrected for payload
        H       % Transformation matrix from the CG to the CO
    end
    properties (Constant)
        g   = 9.81;         % acceleration of gravity (m/s^2)
        rho = 1025;         % density of water (kg/m^3)
        Height = 0.8;       % Height    (m)
        L = 2.0;            % Length (m)
        B = 1.08;           % Beam (m)
        m = 55.0;           % mass (kg)
        rg = [0.2 0 -0.2]'; % CG for hull only (m)
        R44 = 0.4 * Otter.B;      % radii of gyrations (m)
        R55 = 0.25 * Otter.L;
        R66 = 0.25 * Otter.L;
        T_yaw = 1;          % time constant in yaw (s)
        Umax = 6 * 0.5144;  % max forward speed (m/s)
        
        % Data for one pontoon
        B_pont  = 0.25;     % beam of one pontoon (m)
        y_pont  = 0.395;    % distance from centerline to waterline area center (m)
        Cw_pont = 0.75;     % waterline area coefficient (-)
        Cb_pont = 0.4;      % block coefficient, computed from m = 55 kg
        Aw_pont = Otter.Cw_pont * Otter.L * Otter.B_pont;    % waterline area, one pontoon
    end
%%
    methods
        %% Constructor
        function O = Otter(State)
            %Otter(State,Payload) Construct an instance of Otter.
            % State : define a staring position/velocity
            % Payload : Weight and position of payload
            if (nargin < 1)  
                State = zeros(12,1); 
            end

            O.setState(State);
            O.setPayload(0,[0 0 0]');
            O.setCurrent(0,0);
            O.setWind(0,0);
            initPropeller(O)
            waitUntilSteady(O)
            resetHistory(O)
        end
        %% Get - Set
        function P = getPosition(O)
            P = O.State([7 8]);
        end
        function H = getHeading(O)
            H = O.State(12);
        end
        function V = getVelocity(O)
            V = O.State([1 2]);
        end
        function Y = getYawRate(O)
            Y = O.State(6);
        end
        function nu_c = getWaterVelo(O)
            % Calculates the velocity of the vessel relative to the water
            % taking current into account
            u_c = O.Current.v * cos(O.Current.beta - O.State(12));     % current surge velocity
            v_c = O.Current.v * sin(O.Current.beta - O.State(12));     % current sway velocity
            nu_c = [u_c v_c 0 0 0 0]';             % relative velocity vector
        end
        
        function setCurrent(O,v,beta)
            O.Current.v = v;
            O.Current.beta = beta;
            %disp('Current has been updated');
        end
        function setPayload(O,m,r)
            % Set new payload of mass m at postion r = [x y z]'
            O.Payload.m = m;
            if (size(r) == [3 1])
                O.Payload.r = r;
            else
                error('input has wrong dimensions, should be 3x1 vector');
            end
            O.updateStatic()
            disp('Payload and system matrices has been updated');
        end
        function setState(O,s)
            % Set new payload at postion r = [x y z]'
            
            if (size(s) == [12 1])
                O.State = s;
            else
                error('input has wrong dimensions, should be 12x1 vector')
            end
        end
        function setHarbour(O,pos,theta,depth,width)
            O.Harbour.pos = pos;
            O.Harbour.mouth = theta;
            O.Harbour.depth = depth;
            O.Harbour.width = width;
            
            O.Harbour.BernoulliForce = zeros(6,1);
            
            figx = [depth depth 0 0 depth depth 0 0 depth depth];
            figy = [10 width/2+1 width/2+1 width/2 width/2 -width/2 -width/2 -width/2-1 -width/2-1 -10];
            
            O.Harbour.fig = pos+[figx;figy];
        end
        function setWind(O, mean, dir)
            O.Wind.mean = mean;
            O.Wind.dir = dir;
            O.Wind.force = zeros(6,1);
        end
        %% Initialization
        function initPropeller(O)
            % Claculate propeller constants
            P.R = 0.1;
            P.C = 0.05;
            P.angle = pi/8;
            P.blades = 3;
            K = O.rho/2*pi * P.R * P.C * P.blades;
            P.Tnn = K*P.R*sin(P.angle);
            P.Tnv = - K*cos(P.angle);
            
            % Initialize Propeller object state
            P.xi = [0 0]';             % Position of thrusters
            P.n = [0 0]';              % Speed of thrusters
            P.xi_r = [0 0]';           % Position reference
            P.n_r = [0 0]';            % Speed reference
            P.Thrust = [0 0]';
            
            % Set Proppeller object and turn on usage
            O.UseProppeller = true;
            O.Propeller = P;
        end
        function waitUntilSteady(O)
            O.useDistrubance = false;
            O.step(1/100);
            diff = 1;
            lastState = O.State;
            while (diff>1e-6)
                O.step(1/100);
                diff = norm(O.State-lastState);
                lastState = O.State;
            end
            O.useDistrubance = true;
        end
        function resetHistory(O)
            O.t = 0;
            O.History = [];
        end
        
        %% Measurements
        function M = getMeasurement(O)
            x = O.State();
            fromKnots = 1/toKnots(1);
            
            % Furuno Doppler Speed Log DS-80 - accuracy 0.1 knot.
            sd = 0.1 * fromKnots;
            logSpeed = x(1) + sd*randn(1);
            
            % Furuno SC70. differential GPS (DGPS)
            % assuming >5 satellits.
            
            % Compass (Heading)
            sd = deg2rad(0.5);
            HDT = x(12) + sd*randn(1);
            
            % Position (x,y)
            sd = 5/2;                           % 5m approx. (2drms)
            theta = rand(1)*2*pi;
            Rn = [cos(theta); sin(theta)] * sd.*randn(1);
            Position = x(7:8) + Rn;
            
            % VBW
            % sd = 0.2*fromKnots                  % 0.2 knot
            % VBW = norm(x(1:2)) + sd*randn(1);
            
            % Speed over Ground
            sd = 0.02*fromKnots;                  % 0.02 knot
            SoG = norm(x(1:2)) + sd*randn(1);
            
            % Course
            sd = deg2rad(0.4);                  % 0.4 deg rms
            Course = atan2(x(2),x(1)) + sd*randn(1);
            Course = wrapToPi(Course+HDT);
            % Rate of Turn
            sd = deg2rad(0.4);                  % 0.4 deg/s rms
            RoT = x(6) + sd*randn(1);
            
            % Pack
            M.HDT = HDT;
            M.Position = Position;
            M.SOG = SoG;
            M.Course = Course;
            M.RoT = RoT;
        end
        function ym = measurementTransformation(O, M)
            
            SOG = [cos(M.Course-M.HDT); sin(M.Course-M.HDT)]*M.SOG;   % in NED
            ym = [SOG;M.RoT;M.Position;M.HDT];
        end
        
        %% Thrust and Control allocation
        function Ta = controlAllocation(O,Tr,nu)
            l1 = [-0.2; -O.y_pont; 0];                           % lever arm, left propeller (m)
            l2 = [-0.2; O.y_pont; 0];                            % lever arm, right propeller (m)
            B = [eye(3) eye(3); Smtrx(l1) Smtrx(l2)];
            iW = eye(6);
            
            % Pseudo Inverse
%             tau = pinv(B)*Tr;
            % SVD
            [U,S,V] = svd(B*iW*B');
            S(S<eps) = 0;
            iS = S;
            iS(S~=0) = 1./S(S~=0);
            
            C = iW*B'*V*iS*U';
            tau = C*Tr;
            
            
            
            tau1 = tau(1:3);
            tau2 = tau(4:6);
            xi1 = atan2(tau1(2),tau1(1));
            xi2 = atan2(tau2(2),tau2(1));
            a1 = norm(tau1);
            a2 = norm(tau2);
            
            if xi1 > pi/2
                xi1 = xi1-pi;
                a1 = -a1;
            end
            if xi1 < -pi/2
                xi1 = xi1+pi;
                a1 = -a1;
            end
            if xi2 > pi/2
                xi2 = xi2-pi;
                a2 = -a2;
            end
            if xi2 < -pi/2
                xi2 = xi2+pi;
                a2 = -a2;
            end
            
            xi = [xi1 xi2]';
            u = nu(1);
            v = nu(2);
            
            Va = cos(xi)*u + sin(xi)*v;
            
            Tnn = O.Propeller.Tnn;
            Tnv = O.Propeller.Tnv;
            
            % If Va = 0
            %n1 = sign(a1)*sqrt(abs(a1)/O.Propeller.Tnn);
            %n2 = sign(a2)*sqrt(abs(a2)/O.Propeller.Tnn);
            
            n1 = sign(a1)*(sqrt(Tnv^2*Va(1)^2 + 4*Tnn*abs(a1))-sign(a1)*Tnv*Va(1)) / (2*Tnn);
            n2 = sign(a2)*(sqrt(Tnv^2*Va(2)^2 + 4*Tnn*abs(a2))-sign(a2)*Tnv*Va(2)) / (2*Tnn);
            n = [n1 n2]';
            
            O.Propeller.n_r = n;
            O.Propeller.xi_r = xi;
            
            a = Tnn*abs(n).*n + Tnv*abs(n).*Va;
            tau = [ [cos(xi(1)) sin(xi(1)) 0 0 0 0]' [0 0 0 cos(xi(2)) sin(xi(2)) 0]' ]*a;
            Ta = B*tau;
            
            if (abs(Ta(1) - Tr(1)) > 0.5*abs(Tr(1)))
                warning('could not allocate control correctly'); 
            end
            
        end
        function T = updateThrust(O)
            % Propeller data
            l1 = [-0.2; -O.y_pont; 0];                           % lever arm, left propeller (m)
            l2 = [-0.2; O.y_pont; 0];                            % lever arm, right propeller (m)
           
            
            % Propeller forces and moments
            n = O.Propeller.n;
            xi =  O.Propeller.xi;
            
            nu = O.State(1:6);
            nu_c = O.getWaterVelo();
            nu_r = nu - nu_c;
            
            Tnn = O.Propeller.Tnn;
            Tnv = O.Propeller.Tnv;
            
            Va = cos(xi)*nu_r(1) + sin(xi)*nu_r(2);
            thrust = Tnn*abs(n).*n + Tnv*abs(n).*Va;

            O.Propeller.Thrust = thrust;
            
            tau = [ [cos(xi(1)) sin(xi(1)) 0 0 0 0]' [0 0 0 cos(xi(2)) sin(xi(2)) 0]' ]*thrust;
            B = [eye(3) eye(3); Smtrx(l1) Smtrx(l2)];
            T = B * tau;
        end
        
        %% Distrubances
        function tau_wind = windForce(O)

            rho_a = 1.225;        % Density of air (kg/m^3)
            
            nu = O.getVelocity();
            psi = O.getHeading();
            
            V = O.Wind.mean;
            dir = O.Wind.dir;
                        
            V = Rot(dir,2)*[-V;0];
            
            v_w = Rot(psi,2)*V - nu;

            % Area
            Ax = O.Height*O.B;
            Ay = O.Height*O.L;

            % Coefficients
%             disdir = mod(round(dir),360);
%             Cx = C(disdir,1);
%             Cy = C(disdir,2);
%             Ck = C(disdir,3);      
%             Cn = C(disdir,4); 

            Cx = 0.1;
            Cy = 0.3;

            % Force
            Fx =  0.5*rho_a*Cx*Ax*v_w(1)^2*sign(v_w(1));
            Fy =  0.5*rho_a*Cy*Ay*v_w(2)^2*sign(v_w(2));
            tau_wind = [Fx;Fy;0;0;0;0];
            
        end
        function Fb = bernoulliForce(O)
            % Calculate the bernoulli forces in the harbour
                        Fb = zeros(6,1);
            sp = O.getPosition;
            h = O.Harbour;

            lowx = h.pos(1);
            highx = h.pos(1) + h.depth;
            lowy = h.pos(2) - h.width/2;
            highy = h.pos(2) + h.width/2;

            inHabour = inrange(lowx,sp(1),highx) && inrange(lowy,sp(2),highy);
            if inHabour
                Fp = O.Propeller.Thrust;
                Kb = 0.1;

                dp = sp(1) - lowy -O.B/2;
                dsb = highy - sp(1) -O.B/2;

                Fbp     = -Kb * Fp(1) * max(1-dp/O.B,0);
                Fbsb    = Kb * Fp(2) * max(1-dsb/O.B,0);

                Fb(2) = Fbp + Fbsb;
            end
        end
        function tau_ext = externalForces(O)
            % Calculate the external forces acting on the vessel
            Fw = O.windForce();
            O.Wind.force = Fw;
            
            Fb = zeros(6,1);
            if ~isempty(O.Harbour)
                Fb = bernoulliForce(O);
                O.Harbour.BernoulliForce = Fb;

            end
            
            tau_ext =  Fw + Fb;            
        end
        
        %% Model Update
        function updateStatic(O)
            % Mass - Inertia
            Ig_CG = O.m * diag([O.R44^2, O.R55^2, O.R66^2]);                        % only hull in CG
            rg = (O.m*O.rg + O.Payload.m*O.Payload.r)/(O.m+O.Payload.m);            % CG location corrected for payload
            O.Ig = Ig_CG - O.m * Smtrx(rg)^2 - O.Payload.m * Smtrx(O.Payload.r)^2;  % hull + payload in CO
            O.H = Hmtrx(rg);                                                        % Transform MRB and CRB from the CG to the CO

              MRB_CG = [ (O.m+O.Payload.m) * eye(3)  zeros(3,3)
                                         zeros(3,3)          O.Ig     ];
            O.MRB = O.H' * MRB_CG * O.H;
            
            Xudot = 0.1 * O.m;
            Yvdot = 1.5 * O.m;
            Zwdot = 1.0 * O.m;
            Kpdot = 0.2 * O.Ig(1,1);
            Mqdot = 0.8 * O.Ig(2,2);
            Nrdot = 1.7 * O.Ig(3,3);
            
            O.MA = diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);
            O.M = O.MRB + O.MA;
            
            % Hydro statics
            nabla = (O.m+O.Payload.m)/O.rho;                                        % volume of submerged hull
            T = nabla / (2 * O.Cb_pont * O.B_pont*O.L);                             % draft
            I_T = 2 * (1/12)*O.L*O.B_pont^3 * (6*O.Cw_pont^3/((1+O.Cw_pont)*(1+2*O.Cw_pont)))...
                + 2 * O.Aw_pont * O.y_pont^2;
            I_L = 0.8 * 2 * (1/12) * O.B_pont * O.L^3;
            KB = (1/3)*(5*T/2 - 0.5*nabla/(O.L*O.B_pont) );
            BM_T = I_T/nabla;       % BM values
            BM_L = I_L/nabla;
            KM_T = KB + BM_T;       % KM values
            KM_L = KB + BM_L;
            KG = T - rg(3);
            GM_T = KM_T - KG;       % GM values
            GM_L = KM_L - KG;
            
            G33 = O.rho * O.g * (2 * O.Aw_pont);      % spring stiffness
            G44 = O.rho * O.g * nabla * GM_T;
            G55 = O.rho * O.g * nabla * GM_L;
            
            G_CF = diag([0 0 G33 G44 G55 0]);   % spring stiffness matrix in the CF
            LCF = -0.2;
            Hg = Hmtrx([LCF 0 0]);               % transform G_CF from the CF to the CO
            O.G = Hg' * G_CF * Hg;            
            O.draft = T;
            % Natural frequencies
            w3 = sqrt( G33/O.M(3,3) );
            w4 = sqrt( G44/O.M(4,4) );
            w5 = sqrt( G55/O.M(5,5) );
            
            % Linear damping terms (hydrodynamic derivatives)
            Xu = 24.4 * O.g / O.Umax;           % specified using the maximum speed
            Yv = 0;
            Zw = 2 * 0.3 *w3 * O.M(3,3);      % specified using relative damping factors
            Kp = 2 * 0.2 *w4 * O.M(4,4);
            Mq = 2 * 0.4 *w5 * O.M(5,5);
            Nr = O.M(6,6) / O.T_yaw;            % specified using the time constant in T_yaw
            
            O.DL = diag([Xu Yv Zw Kp Mq Nr]);
        end
        function updateDynamic(O,nu_r)
            % C
            nu2 = nu_r(4:6);
            CRB_CG = [ (O.m+O.Payload.m) * Smtrx(nu2)         zeros(3,3)
                                           zeros(3,3)   -Smtrx(O.Ig*nu2)  ];
            O.CRB = O.H' * CRB_CG * O.H;
            O.CA  = m2c(O.MA, nu_r);
            O.CA(6,1) = 0; % Assume that the Munk moment in yaw can be neglected
            O.CA(6,2) = 0; % These terms, if nonzero, must be balanced by adding nonlinear damping
            O.C = O.CRB + O.CA;
            % D
            Nr = O.DL(6,6);
            O.DNL = diag([0 0 0 0 0 Nr*10 * abs(nu_r(6))]);
            O.D = O.DL + O.DNL;
            
        end
        function xdot = getStateDerivative(O)
            % Get the time derivative of the state vector 
            % State and current variables
            nu = O.State(1:6);  
            eta = O.State(7:12);                              % positions
            
            nu_c = O.getWaterVelo();
            nu_r = nu - nu_c;
            
            O.updateDynamic(nu_r)
            
            % Kinematics
            J = eulerang(eta(4),eta(5),eta(6));
            
            % Thrust
            if O.UseProppeller
                O.Thrust = O.updateThrust();
            end
            tau = O.Thrust;
            
            % Strip theory: cross-flow drag integrals for Yh and Nh
            tau_crossflow = crossFlowDrag(O.L,O.B_pont,O.draft,nu_r);
            
            tau_ext = zeros(6,1);
            if O.useDistrubance
                tau_ext = O.externalForces();
            end
            
        
            % Calculate the time derivative of the state vector
%             nudot = O.M \ ( tau - O.D*nu_r + tau_crossflow - O.C * nu_r - O.G * eta);
%             etadot = J * nu + nu_c;
%             xdot = [nudot; etadot];

            nudot = O.M \ ( tau + tau_crossflow + tau_ext - O.CRB*nu - O.CA*nu_r - O.D*nu_r - O.G*eta);
            etadot = J * nu;
            xdot = [nudot; etadot];
            
            
        end
        %% Step
        function step(O,Ts)
            % Progress time for the Otter
            O.t = O.t + 1;
            O.State = O.State + Ts * O.getStateDerivative;
            O.State(10:12) = wrapToPi(O.State(10:12));          % angles are represented in the interval [-pi;pi];
            
            stepPropeller(O,Ts)

            % Save route
            O.History.Velo(:,O.t) = O.State(1:6);
            O.History.Pos(:,O.t) = O.State(7:12);
            O.History.Propeller.state(:,O.t) = [O.Propeller.xi; O.Propeller.n];
            O.History.Propeller.ref(:,O.t) = [O.Propeller.xi_r; O.Propeller.n_r];
            O.History.Propeller.thrust(:,O.t) = O.Propeller.Thrust;
            O.History.Thrust(:,O.t) = O.Thrust;
            if ~isempty( O.Harbour), O.History.Bernoulli(:,O.t) = O.Harbour.BernoulliForce; end
            O.History.WindForce(:,O.t) = O.Wind.force;
        end
        function stepPropeller(O,Ts)
            % Progress time for proppeller, this will make the proppeller
            % state go towards the proppeller setpoint
           
            n_max =  100;    % maximum propeller rev. (rad/s)
            n_min = -90;    % minimum propeller rev. (rad/s)
            
            dn =    2*(O.Propeller.n_r - O.Propeller.n);
            O.Propeller.n = O.Propeller.n + Ts*dn;
            % Propeller forces and moments
            O.Propeller.n(O.Propeller.n>n_max) = n_max;             % saturation, physical limits
            O.Propeller.n(O.Propeller.n<n_min) = n_min;             % saturation, physical limits
            
            dxi =   5*(O.Propeller.xi_r - O.Propeller.xi);
%             dximax = deg2rad(20);
%             dxi(dxi>dximax) = dximax;
%             dxi(dxi<-dximax) = -dximax;
            O.Propeller.xi = O.Propeller.xi + Ts*dxi;
        end
        %% Visualization
        function plot(O, T, Course)
            title = 'Course';
            niceplot(O.History.Pos(1,:),O.History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'west');
            axis equal
            grid on
            set(gca, 'YDir','reverse')
            if ~isempty(O.Harbour)
                plot(O.Harbour.fig(1,:), O.Harbour.fig(2,:), 'k-', 'LineWidth', 4);
            end
            color = 'g-';
            for i = [1:2000:O.t, O.t]
                vessel = vesselplot(O.History.Pos(6,i),O.History.Propeller.state(1:2,i));
                if i == O.t
                    color = 'r-';
                end
                plot(vessel(1,:)+O.History.Pos(1,i), vessel(2,:)+O.History.Pos(2,i), color, 'LineWidth', 2);
                color = 'b-';
            end
            
            if (nargin > 2) && iscell(Course) && isa(Course{1}, 'WayPoint')
                    WP = Course;
                arrow = [0 2 0 0; 1 0 -1 1]*0.5;
                x = 0:0.1:2*pi;
                for it = 1:length(WP)
                    pos = WP{it}.pos;
                    h = -WP{it}.heading;
                    ad = WP{it}.accept.distance;
                    R = [cos(h) sin(h); -sin(h) cos(h)];
                    Rarrow = R*arrow+pos;
                    plot(pos(1),pos(2),'ko','LineWidth', 4);
                    plot(Rarrow(1,:),Rarrow(2,:),'k','LineWidth', 4);
                    plot(cos(x)*ad+pos(1),sin(x)*ad+pos(2),'k','LineWidth', 4);
                end
            elseif (nargin > 2)
                 P = Course;
                 plot(P(1,:),P(2,:),'k-','LineWidth', 2);
            end

            title = 'Linear Velocities';
            names = ["$u$", "$v$", "$w$"];
            niceplot(T,toKnots(O.History.Velo(1:3,:)), names, title, ["-"], ["time [s]", "[knot]"], 'north');
            
            title = 'Angular Velocities';
            names = ["$p$", "$q$", "$r$"];
            niceplot(T, rad2deg(O.History.Velo(4:6,:))*60, names, title, ["-"], ["time [s]", "[deg/min]"], 'northeast');
            ytickformat('%.0f°')
            
            
            title = 'Position';
            names = ["$x$ north", "$y$ east", "$z$ down"];
            niceplot(T,O.History.Pos(1:3,:), names, title, ["-"], ["time [s]", "[m]"], 'south');
            
            title = 'Orientation';
            names = ["$\phi$ roll", "$\theta$ pitch", "$\psi$ yaw"];
            niceplot(T,rad2deg(O.History.Pos(4:6,:)), names, title, ["-"], ["time [s]", ""], 'southeast');
            yticks(-180:30:180)
            tl = [180:30:359 0:30:180]+"°"; tl(7) = "N"; tl(10) = "E"; tl([1,13]) = "S"; tl(4) = "W";
            yticklabels(tl)
            
        end
        function plotPropeller(O, T)
            if (O.UseProppeller)
                title = 'Propeller angle';
                names = ["P", "SB","P", "SB"];
                niceplot(T, rad2deg([O.History.Propeller.ref(1:2,:); O.History.Propeller.state(1:2,:)]), names, title, ["--","-"], ["time [s]", ""], 'west');
                ytickformat('%.0f°')
                
                title = 'Propeller velocity';
                names = ["P", "SB","P", "SB"];
                niceplot(T, toRPM([O.History.Propeller.ref(3:4,:); O.History.Propeller.state(3:4,:)]), names, title, ["--","-"], ["time [s]", "[rpm]"], 'center');
                
                title = 'Propeller Thrust';
                names = ["P", "SB"];
                niceplot(T, O.History.Propeller.thrust, names, title, ["r-","g-"], ["time [s]", "[N]"], 'east');
            else
                title = 'Thrust';
                names = ["$\tau_u$", "$\tau_v$", "$\tau_r$"];
                niceplot(T, O.History.Thrust([1 2 6],:), names, title, ["-"], ["time [s]", "[N]"], 'southwest');
            end
        end
        function plotDistrubance(O, T)
            if ~isempty( O.Harbour) 
            title = 'Bernoulli force';
            names = ["$F_bv$"];
            niceplot(T,O.History.Bernoulli(2,:), names, title, ["-"], ["time [s]", "$[N]$"], 'northeast'); 
            end
            title = 'Wind force';
            names = ["$F_wu$","$F_wv$"];
            niceplot(T,O.History.WindForce(1:2,:), names, title, ["-"], ["time [s]", "$[N]$"], 'southeast');
         end
    end
end

