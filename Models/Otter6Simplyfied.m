classdef Otter6Simplyfied < Vessel
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
        R44 = 0.4 * Otter6.B;      % radii of gyrations (m)
        R55 = 0.25 * Otter6.L;
        R66 = 0.25 * Otter6.L;
        T_yaw = 1;          % time constant in yaw (s)
        Umax = 6 * 0.5144;  % max forward speed (m/s)
        
        % Data for one pontoon
        B_pont  = 0.25;     % beam of one pontoon (m)
        y_pont  = 0.395;    % distance from centerline to waterline area center (m)
        Cw_pont = 0.75;     % waterline area coefficient (-)
        Cb_pont = 0.4;      % block coefficient, computed from m = 55 kg
        Aw_pont = Otter6.Cw_pont * Otter6.L * Otter6.B_pont;    % waterline area, one pontoon
    end
    methods
        function O = Otter6Simplyfied(State,Payload)
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
            
            nu_r = O.getWaterVelo();
            
            Tnn = O.Propeller.Tnn;
            Tnv = O.Propeller.Tnv;
            
            Va = cos(xi)*nu_r(1) + sin(xi)*nu_r(2);
            Thrust = Tnn*abs(n).*n + Tnv*abs(n).*Va;
            O.Prop.Thrust = Thrust;
            
            tau = [ [cos(xi(1)) sin(xi(1)) 0 0 0 0]' [0 0 0 cos(xi(2)) sin(xi(2)) 0]' ]*Thrust;
            B = [eye(3) eye(3); Smtrx(l1) Smtrx(l2)];
            T = B * tau;
        end
        function nu_r = getWaterVelo(O)
            % Calculates the velocity of the vessel relative to the water
            % taking current into account
            u_c = O.Current.v * cos(O.Current.beta - O.State(12));     % current surge velocity
            v_c = O.Current.v * sin(O.Current.beta - O.State(12));     % current sway velocity
            nu_r = O.State(1:6) - [u_c v_c 0 0 0 0]';             % relative velocity vector
        end
        
        
        function xdot = getStateDerivative(O,t,x)
            
            nu = O.State(1:6);
            eta = O.State(7:12);
            u = nu(1);
            v = nu(2);
            w = nu(3);
            p = nu(4);
            q = nu(5);
            r = nu(6);
            
            nu_r = nu;
            
            MRB = [
                55.0000         0         0         0  -11.0000         0
                0   55.0000         0   11.0000         0   11.0000
                0         0   55.0000         0  -11.0000         0
                0   11.0000         0   14.6643         0    4.4000
                -11.0000         0  -11.0000         0   22.5500         0
                0   11.0000         0    4.4000         0   18.1500
                ];
            
            MA =    [
                
            5.5000         0         0         0         0         0
            0   82.5000         0         0         0         0
            0         0   55.0000         0         0         0
            0         0         0    2.4929         0         0
            0         0         0         0   14.5200         0
            0         0         0         0         0   27.1150
            ];
        
        M = MRB + MA;
        
        CRB = [
            [    0,       -55*r,  55*q,                  -11*r,                  -11*q,                -11*r]
            [ 55*r,           0, -55*p,                      0,            11*p - 11*r,                    0]
            [-55*q,        55*p,     0,                   11*p,                   11*q,                 11*p]
            [ 11*r,           0, -11*p,                      0,   4.4000*p + 13.7500*r,           -18.1500*q]
            [ 11*q, 11*r - 11*p, -11*q, - 4.4000*p - 13.7500*r,                      0, 10.2643*p + 4.4000*r]
            [ 11*r,           0, -11*p,              18.1500*q, - 10.2643*p - 4.4000*r,                    0]
            ];
        
        
        CA = [
            [    0,    0,          0,          0,      55*w, -82.5000*v]
            [    0,    0,          0,      -55*w,         0,   5.5000*u]
            [    0,    0,          0,  82.5000*v, -5.5000*u,          0]
            [    0, 55*w, -82.5000*v,          0, 27.1150*r, -14.5200*q]
            [-55*w,    0,   5.5000*u, -27.1150*r,         0,   2.4929*p]
            [    0,    0,          0,  14.5200*q, -2.4929*p,          0]
            ];
        
        C = CRB + CA;
        
        G = 1.0e+03*[
            0         0         0         0         0         0
            0         0         0         0         0         0
            0         0    7.5414         0    1.5083         0
            0         0         0    1.0773         0         0
            0         0    1.5083         0    2.8534         0
            0         0         0         0         0         0
            ];
        
        
        D = [
            [77.5544,                                                                     0,        0,       0,        0,                                                               0]
            [      0, 1.7764e-15*sign(v)*(7.4603e+15*r + 1.3043e+17*v + 2.8797e+15*sign(v)),        0,       0,        0,    35.9154*abs(r) + 13.2522*abs(v) + 35.9154*r*sign(r) - 4.8994]
            [      0,                                                                     0, 546.4805,       0,        0,                                                               0]
            [      0,                                                                     0,        0, 54.3823,        0,                                                               0]
            [      0,                                                                     0,        0,       0, 246.0496,                                                               0]
            [      0, 8.8818e-16*sign(v)*(8.8677e+16*r + 1.1887e+16*v - 4.6509e+15*sign(v)),        0,       0,        0, 460.1313*abs(r) + 78.7613*abs(v) + 460.1313*r*sign(r) + 49.3007]
            ];
        
        % Kinematics
        J = eulerang(eta(4),eta(5),eta(6));
        
        %Thrust
        if O.UseProppeller
            O.Thrust = O.updateThrust();
        end
        tau = O.Thrust;
        
        % Time derivative of the state vector
        nudot = M \ ( tau - C * nu_r - D * nu_r - G * eta);
        etadot = J * nu ;
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

