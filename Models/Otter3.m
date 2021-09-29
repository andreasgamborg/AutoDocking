classdef Otter3 < Vessel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
        A
        B
        UseProppeller % boolean for switching between tau or [n xi] as actuator
    end
    properties (Constant)
        
        y_pont  = 0.395;    % distance from centerline to waterline area center (m)
        
    end
    methods
        function O = Otter3(path,State,Payload)
            %Otter6(State,Payload) Construct an instance of Otter6
            if (nargin < 1), error('Provide a path for system matrices A & B'); end
            if (nargin < 2),  State = zeros(6,1); end
            if (nargin < 3)
                Payload.m = 0;
                Payload.r = [0;0;0];
            end
            O = O@Vessel(State,Payload);
            load(path);
            if isnumeric(A) && isnumeric(B)
                O.A = A;
                O.B = B;
            else
                error('System matrices must be numeric')
            end
            O.UseProppeller = 0;
            O.Prop.xi = [0 0]';
            O.Prop.n = [0 0]';
            O.Prop.Thrust = [0 0]';
        end
        function P = getPos(O)
            P = O.State([4 5 6]);
        end
        function T = updateThrust(O)
            % Propeller data
            l1 = [0; -O.y_pont; 0];                           % lever arm, left propeller (m)
            l2 = [0; O.y_pont; 0];                            % lever arm, right propeller (m)
            k_pos = 0.02216/2;                      % Positive Bollard, one propeller
            k_neg = 0.01289/2;                      % Negative Bollard, one propeller
            n_max =  sqrt((0.5*24.4 * O.g)/k_pos);    % maximum propeller rev. (rad/s)
            n_min = -sqrt((0.5*13.6 * O.g)/k_neg);    % minimum propeller rev. (rad/s)
            
            Propeller.R = 0.1;
            Propeller.C = 0.05;
            Propeller.angle = pi/8;
            Propeller.blades = 3;
            Propeller.K = O.rho/2*pi*Propeller.R*Propeller.C*Propeller.blades;
            
            % Propeller forces and moments
            n = O.Prop.n;
            xi =  O.Prop.xi;
            
            % Propeller forces and moments
            n(n>n_max) = n_max;             % saturation, physical limits
            n(n<n_min) = n_min;             % saturation, physical limits
            
            Thrust = Propeller.K*Propeller.R*sin(Propeller.angle)*n.*abs(n) ...
                - Propeller.K*cos(Propeller.angle)*abs(n)*O.State(1);
            O.Prop.Thrust = Thrust;
            
            Thrust = [Thrust(1) 0 ; 0 Thrust(2)];
            
            tau1 = [cos(xi)'; sin(xi)'; zeros(1,length(xi))] * Thrust;
            tau2 = Smtrx(l1)*tau1(:,1) + Smtrx(l2)*tau1(:,2);
            T = [sum(tau1,2); tau2];
            
        end
        function xdot = getStateDerivative(O)
            nu = O.State(1:3);                          % velocities
            pos = O.State(4:6);                         % positions
            
            u_c = O.Current.v * cos(O.Current.beta);     % current velocity NED
            v_c = O.Current.v * sin(O.Current.beta);     % current velocity NED
            
            %Thrust
            if O.UseProppeller
                O.Thrust = O.updateThrust();
            end
            tau = O.Thrust;
            
            % Kinematics
            psi = O.State(6);
            R = [   cos(psi)   -sin(psi)    0
                sin(psi)    cos(psi)    0
                0           0           1 ];
            % Time derivative of the state vector
            nudot = O.A*nu + O.B*tau([1 2 6]);
            etadot = R * nu + [u_c v_c 0]';
            xdot = [nudot; etadot];
            
        end
        function step(O,Ts)
            O.t = O.t + 1;
            O.State = O.State + Ts * O.getStateDerivative();
            O.State(6) = wrapToPi(O.State(6));
            
            % Save route
            O.History.Velo(:,O.t) = O.State(1:3);
            O.History.Pos(:,O.t) = O.State(4:6);
            O.History.Propeller(:,O.t) = [O.Prop.xi; O.Prop.n; O.Prop.Thrust];
            O.History.Thrust(:,O.t) = O.Thrust;
        end
        function plot(V,T)
            title = 'Course';
            niceplot(V.History.Pos(1,:),V.History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'north');
            axis equal
            grid
            set(gca, 'YDir','reverse')
            
            color = 'g-';
            for i = [1:500:V.t, V.t]
                
                vessel = vesselplot(V.History.Pos(3,i),V.History.Propeller(1:2,i));
                
                if i == V.t
                    color = 'r-';
                end
                
                plot(vessel(1,:)+V.History.Pos(1,i), vessel(2,:)+V.History.Pos(2,i), color, 'LineWidth', 2);
                
                color = 'b-';
            end
            
            
            title = 'Linear Velocities';
            names = ["$u$ surge", "$v$ sway"];
            niceplot(T,toKnots(V.History.Velo(1:2,:),'m/s'), names, title, ["--"], ["time [s]", "[knot]"], 'northeast');
            
            title = 'Orientation';
            names = ["$\psi$ yaw"];
            if(1)
                niceplot(T,rad2deg(V.History.Pos(3,:)), names, title, ["--"], ["time [s]", ""], 'south');
                yticks(-180:30:180)
                tl = [180:30:359 0:30:180]+"°"; tl(7) = "N"; tl(10) = "E"; tl([1,13]) = "S"; tl(4) = "W";
                yticklabels(tl)
            else
                niceplot(T,V.History.Pos(4:6,:), names, title, ["--"], ["time [s]", "[rad]"], 'south');
            end
            
            title = 'Angular Velocities';
            names = ["$r$ yaw"];
            if(1)
                niceplot(T,rad2deg(V.History.Velo(3,:)*60), names, title, ["--"], ["time [s]", "[deg/min]"], 'southeast');
                ytickformat('%.0f°')
            else
                niceplot(T,V.History.Velo(4:6,:), names, title, ["--"], ["time [s]", "[rad/s]"], 'southeast');
            end
            
            title = 'Propeller velocity';
            names = ["P", "SB"];
            niceplot(T,toRPM(V.History.Propeller(3:4,:)), names, title, ["-"], ["time [s]", "[rpm]"], 'northwest');
            title = 'Propeller Thrust';
            names = ["P", "SB"];
            niceplot(T,V.History.Propeller(5:6,:), names, title, ["r-","g--"], ["time [s]", "[N]"], 'southwest');
        end
    end
end

