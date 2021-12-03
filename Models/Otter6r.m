classdef Otter6r < Vessel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
        trim_moment = 0;
        trim_setpoint = 280;
        UseProppeller % boolean for switching between tau or [n xi] as actuator
        Propeller
    end
    properties (Constant)
        L = 2.0;            % length (m)
        B = 1.08;           % beam (m)
        m = 55.0;           % mass (kg)
        rg = [0.2 0 -0.2]'; % CG for hull only (m)
        R44 = 0.4 * Otter6r.B;      % radii of gyrations (m)
        R55 = 0.25 * Otter6r.L;
        R66 = 0.25 * Otter6r.L;
        T_yaw = 1;          % time constant in yaw (s)
        Umax = 6 * 0.5144;  % max forward speed (m/s)
        
        % Data for one pontoon
        B_pont  = 0.25;     % beam of one pontoon (m)
        y_pont  = 0.395;    % distance from centerline to waterline area center (m)
        Cw_pont = 0.75;     % waterline area coefficient (-)
        Cb_pont = 0.4;      % block coefficient, computed from m = 55 kg
        Aw_pont = Otter6r.Cw_pont * Otter6r.L * Otter6r.B_pont;    % waterline area, one pontoon
    end
    methods
        function O = Otter6r(State,Payload)
            %Otter6(State,Payload) Construct an instance of Otter6
            if (nargin < 1),  State = zeros(12,1); end
            if (nargin < 2)
                Payload.m = 0;
                Payload.r = [0;0;0];
            end
            O = O@Vessel(State,Payload);
            O.UseProppeller = true;
            O.Prop.xi = [0 0]';
            O.Prop.n = [0 0]';
            O.Prop.xi_r = [0 0]';
            O.Prop.n_r = [0 0]';
            O.Prop.Thrust = [0 0]';
            O.Propeller = O.calculatePropeller;
            waitUntilSteady(O)
            resetHistory(O)
        end
        function resetHistory(O)
            O.t = 0;
            O.History = [];
        end
        function waitUntilSteady(O)
            diff = 1;
            lastState = O.State;
            while (diff>1e-6)
                O.step(1/100);
                O.step(1/100);
                diff = norm(O.State-lastState);
                lastState = O.State;
            end
            O.t = 0;
        end
        function P = getPos(O)
            P = O.State([7 8 12]);
        end
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
        function Ta = controlAllocation(O,Tr,nu)
            
            l1 = [-0.2; -O.y_pont; 0];                           % lever arm, left propeller (m)
            l2 = [-0.2; O.y_pont; 0];                            % lever arm, right propeller (m)
            B = [eye(3) eye(3); Smtrx(l1) Smtrx(l2)];
            iW = eye(6);
            
            %             tau = pinv(B)*T;
            %% SVD
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
            
            %             n1 = sign(a1)*sqrt(abs(a1)/O.Propeller.Tnn);
            %             n2 = sign(a2)*sqrt(abs(a2)/O.Propeller.Tnn);
            
            n1 = sign(a1)*(sqrt(Tnv^2*Va(1)^2 + 4*Tnn*abs(a1))-Tnv*Va(1)) / (2*Tnn);
            n2 = sign(a2)*(sqrt(Tnv^2*Va(2)^2 + 4*Tnn*abs(a2))-Tnv*Va(2)) / (2*Tnn);
            n = [n1 n2]';
            
            O.Prop.n_r = n;
            O.Prop.xi_r = xi;
            
            a = Tnn*abs(n).*n + Tnv*abs(n).*Va;
            tau = [ [cos(xi(1)) sin(xi(1)) 0 0 0 0]' [0 0 0 cos(xi(2)) sin(xi(2)) 0]' ]*a;
            Ta = B*tau;
            if (Ta ~= Tr), warning('could not allocate control correctly'); end
            
        end
        function Propeller = calculatePropeller(O)
            Propeller.R = 0.1;
            Propeller.C = 0.05;
            Propeller.angle = pi/8;
            Propeller.blades = 3;
            K = O.rho/2*pi*Propeller.R*Propeller.C*Propeller.blades;
            Propeller.Tnn = K*Propeller.R*sin(Propeller.angle);
            Propeller.Tnv = - K*cos(Propeller.angle);
        end
        function T = updateThrust(O)
            % Propeller data
            l1 = [-0.2; -O.y_pont; 0];                           % lever arm, left propeller (m)
            l2 = [-0.2; O.y_pont; 0];                            % lever arm, right propeller (m)
           
            
            % Propeller forces and moments
            n = O.Prop.n;
            xi =  O.Prop.xi;
            
            nu = O.State(1:6);
            nu_c = O.getWaterVelo();
            nu_r = nu - nu_c;
            
            Tnn = O.Propeller.Tnn;
            Tnv = O.Propeller.Tnv;
            
            Va = cos(xi)*nu_r(1) + sin(xi)*nu_r(2);
            Thrust = Tnn*abs(n).*n + Tnv*abs(n).*Va;
            O.Prop.Thrust = Thrust;
            
            tau = [ [cos(xi(1)) sin(xi(1)) 0 0 0 0]' [0 0 0 cos(xi(2)) sin(xi(2)) 0]' ]*Thrust;
            B = [eye(3) eye(3); Smtrx(l1) Smtrx(l2)];
            T = B * tau;
        end
        function nu_c = getWaterVelo(O)
            % Calculates the velocity of the vessel relative to the water
            % taking current into account
            u_c = O.Current.v * cos(O.Current.beta - O.State(12));     % current surge velocity
            v_c = O.Current.v * sin(O.Current.beta - O.State(12));     % current sway velocity
            nu_c = [u_c v_c 0 0 0 0]';             % relative velocity vector
        end
        
        
        function xdot = getStateDerivative(O,t,x)
            % State and current variables
            nu = O.State(1:6);  nu1 = O.State(1:3); nu2 = O.State(4:6);   % velocities
            eta = O.State(7:12);                              % positions
            U = sqrt(nu(1)^2 + nu(2)^2 + nu(3)^2);      % speed
            
            nu_c = O.getWaterVelo();
            %nu_r = nu - nu_c;
            
            % Inertia dyadic, volume displacement and draft
            nabla = (O.m+O.Payload.m)/O.rho;                         % volume
            T = nabla / (2 * O.Cb_pont * O.B_pont*O.L);       % draft
            Ig_CG = O.m * diag([O.R44^2, O.R55^2, O.R66^2]);    % only hull in CG
            rg = (O.m*O.rg + O.Payload.m*O.Payload.r)/(O.m+O.Payload.m);           % CG location corrected for payload
            Ig = Ig_CG - O.m * Smtrx(rg)^2 - O.Payload.m * Smtrx(O.Payload.r)^2;  % hull + payload in CO
            
            
            % MRB and CRB (Fossen 2021)
            I3 = eye(3);
            O3 = zeros(3,3);
            
            MRB_CG = [ (O.m+O.Payload.m) * I3  O3
                O3           Ig ];
            CRB_CG = [ (O.m+O.Payload.m) * Smtrx(nu2)         O3
                O3               -Smtrx(Ig*nu2)  ];
            
            H = Hmtrx(rg);              % Transform MRB and CRB from the CG to the CO
            MRB = H' * MRB_CG * H;
            CRB = H' * CRB_CG * H;
            
            % Hydrodynamic added mass (best practise)
            Xudot = -0.1 * O.m;
            Yvdot = -1.5 * O.m;
            Zwdot = -1.0 * O.m;
            Kpdot = -0.2 * Ig(1,1);
            Mqdot = -0.8 * Ig(2,2);
            Nrdot = -1.7 * Ig(3,3);
            
            MA = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);
            CA  = m2c(MA, nu);
            CA(6,1) = 0; % Assume that the Munk moment in yaw can be neglected
            CA(6,2) = 0; % These terms, if nonzero, must be balanced by adding nonlinear damping
            
            % System mass and Coriolis-centripetal matrices
            M = MRB + MA;
            C = CRB + CA;
            
            % Hydrostatic quantities (Fossen 2021)
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
            H = Hmtrx([LCF 0 0]);               % transform G_CF from the CF to the CO
            G = H' * G_CF * H;
            
            % Natural frequencies
            w3 = sqrt( G33/M(3,3) );
            w4 = sqrt( G44/M(4,4) );
            w5 = sqrt( G55/M(5,5) );
            
            % Linear damping terms (hydrodynamic derivatives)
            Xu = -24.4 * O.g / O.Umax;           % specified using the maximum speed
            Yv = 0;
            Zw = -2 * 0.3 *w3 * M(3,3);      % specified using relative damping factors
            Kp = -2 * 0.2 *w4 * M(4,4);
            Mq = -2 * 0.4 *w5 * M(5,5);
            Nr = -M(6,6) / O.T_yaw;            % specified using the time constant in T_yaw
            
            
            % Linear damping using relative velocities + nonlinear yaw damping
            Xh = Xu * nu(1);
            Yh = Yv * nu(2);
            Zh = Zw * nu(3);
            Kh = Kp * nu(4);
            Mh = Mq * nu(5);
            Nh = Nr * (1 + 10 * abs(nu(6))) * nu(6);
            
            tau_damp = [Xh Yh Zh Kh Mh Nh]';
            
            % Strip theory: cross-flow drag integrals for Yh and Nh
            tau_crossflow = crossFlowDrag(O.L,O.B_pont,T,nu);
            
            % Ballast
            g_0 = [0 0 0 0 O.trim_moment 0]';
            
            % Kinematics
            J = eulerang(eta(4),eta(5),eta(6));
            
            %Thrust
            if O.UseProppeller
                O.Thrust = O.updateThrust();
            end
            tau = O.Thrust;
            
            % Time derivative of the state vector
            nudot = M \ ( tau + tau_damp + tau_crossflow - C * nu - G * eta - g_0);
            etadot = J * nu + nu_c;
            xdot = [nudot; etadot];
            
        end
        function step(O,Ts)
            O.t = O.t + 1;
            O.State = O.State + Ts * O.getStateDerivative;
            
            %             [~,X] = ode45(@O.getStateDerivative,[0 Ts], O.State);
            %             O.State = X(end,:)';
            
            O.State(10:12) = wrapToPi(O.State(10:12));
            O.trim_moment = O.trim_moment + 0.05 * (O.trim_setpoint - O.trim_moment);
            
            
            k_pos = 0.02216/2;                      % Positive Bollard, one propeller
            k_neg = 0.01289/2;                      % Negative Bollard, one propeller
            n_max =  sqrt((0.5*24.4 * O.g)/k_pos);    % maximum propeller rev. (rad/s)
            n_min = -sqrt((0.5*13.6 * O.g)/k_neg);    % minimum propeller rev. (rad/s)
            dn =    0.5*(O.Prop.n_r - O.Prop.n);
            O.Prop.n = O.Prop.n + Ts*dn;
            % Propeller forces and moments
            O.Prop.n(O.Prop.n>n_max) = n_max;             % saturation, physical limits
            O.Prop.n(O.Prop.n<n_min) = n_min;             % saturation, physical limits
            
            dxi =   3*(O.Prop.xi_r - O.Prop.xi);
            dximax = deg2rad(10);
            dxi(dxi>dximax) = dximax;
            dxi(dxi<-dximax) = -dximax;
            O.Prop.xi = O.Prop.xi + Ts*dxi;
            
            % Save route
            O.History.Velo(:,O.t) = O.State(1:6);
            O.History.Pos(:,O.t) = O.State(7:12);
            O.History.Propeller.state(:,O.t) = [O.Prop.xi; O.Prop.n];
            O.History.Propeller.ref(:,O.t) = [O.Prop.xi_r; O.Prop.n_r];
            O.History.Propeller.thrust(:,O.t) = O.Prop.Thrust;
            O.History.Thrust(:,O.t) = O.Thrust;
        end
        function plot(O, T, Course)
            title = 'Course';
            niceplot(O.History.Pos(1,:),O.History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'west');
            axis equal
            grid
            set(gca, 'YDir','reverse')
            
            color = 'g-';
            for i = [1:1000:O.t, O.t]
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
    end
end

