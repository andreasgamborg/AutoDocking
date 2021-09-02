classdef Ferry3 < Vessel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        L = 6.0;            % length (m)
        B = 2.0;            % beam (m)
        m = 200.0;          % mass of hull (kg)
        rg = [0.2 0]';      % CG for hull only (m)
        Gy_yaw = 1.5;  % radii of gyrations (m)
        
        % Data for one pontoon
        B_pont  = 0.3;     % beam of one pontoon (m)
        y_pont  = 0.3;    % distance from centerline to waterline area center (m)
        Cb_pont = 0.4;      % block coefficient, computed from m = 55 kg
        
    end
    methods
        function F = Ferry3(State,Payload)
            %Ferry3(State,Payload) Construct an instance of Ferry3
            if nargin < 1,  State = zeros(6,1); end
            if nargin < 2
                Payload.m = 0;
                Payload.r = [0;0;0];
            end
            F = F@Vessel(State,Payload);
        end
        
        
        function T = getThrust(V)
            % Propeller data
            l1 = [-V.L*0.4;-V.y_pont];                  % position of port propeller (m)
            l2 = [-V.L*0.4;V.y_pont];                   % position of starboard propeller (m)
            n_max =  60;                            % maximum propeller rev. (rad/s)
            n_min = -60;                            % minimum propeller rev. (rad/s)
            Propeller.R = 0.1;
            Propeller.C = 0.05;
            Propeller.angle = pi/8;
            Propeller.blades = 3;
            Propeller.K = V.rho/2*pi*Propeller.R*Propeller.C*Propeller.blades;
            
            % Propeller forces and moments
            n = V.Prop.n;
            xi =  V.Prop.xi;
            
            n(n>n_max) = n_max;             % saturation, physical limits
            n(n<n_min) = n_min;
            
            nu_r = V.getWaterVelo();
            
            Thrust = Propeller.K*Propeller.R*sin(Propeller.angle)*n.*abs(n) ...
                - Propeller.K*cos(Propeller.angle)*abs(n)*nu_r(1);
            V.Prop.Thrust = Thrust;
            
            Thrust = [Thrust(1) 0 ; 0 Thrust(2)];
            
            tau1 = [cos(xi)'; sin(xi)'] * Thrust;
            
            l1_h = [-l1(2); l1(1)];
            l2_h = [-l2(2); l2(1)];
            
            y1 = dot(tau1(:,1),l1_h) / norm(l1_h);
            y2 = dot(tau1(:,2),l2_h) / norm(l2_h);
            
            tau2 = y1 + y2;
            T = [sum(tau1,2); tau2];
            
        end
        function nu_r = getWaterVelo(Vessel)
            u_c = Vessel.Current.v * cos(Vessel.Current.beta - Vessel.State(6));     % current surge velocity
            v_c = Vessel.Current.v * sin(Vessel.Current.beta - Vessel.State(6));     % current sway velocity
            nu_r = Vessel.State(1:3) - [u_c v_c 0]';                        % relative velocity vector
        end
        function xdot = getStateDerivative(Vessel,t)
            m = Vessel.m;
            nu = Vessel.State(1:3);                          % velocities
            pos = Vessel.State(4:6);                         % positions
            
            % current
            u_c = Vessel.Current.v * cos(Vessel.Current.beta - pos(3));           % current surge velocity
            v_c = Vessel.Current.v * sin(Vessel.Current.beta - pos(3));           % current sway velocity
            nu_r = nu - [u_c v_c 0]';             % relative velocity vector
            
            % Inertia dyadic, volume displacement and draft
            Iz = m * Vessel.Gy_yaw^2;                                  % only hull in CG
            
            % Rigid Body kinetics
            M = [   m       0       0
                0       m       m*Vessel.rg(1)
                0       m*Vessel.rg(1) Iz      ];
            
            Cvr = m*(Vessel.rg(1)*nu(3)+nu(2));
            C = [   0       0      -Cvr
                0       0       m*nu(1)
                Cvr    -m*nu(1) 0       ];
            
            D = [   10      0       0
                0       100     1
                0       1       100     ];
            
            %Thrust
            tau = Vessel.getThrust();
            % Kinematics
            psi = Vessel.State(6);
            R = [   cos(psi)   -sin(psi)    0
                sin(psi)    cos(psi)    0
                0           0           1 ];
            
            % Time derivative of the state vector
            nudot = M \ ( tau - C * nu - D * nu_r);
            etadot = R * nu ;
            xdot = [nudot; etadot];
        end
        function step(V,Ts)
            V.t = V.t + 1;
            V.State = V.State + Ts * V.getStateDerivative();
            V.State(6) = wrapToPi(V.State(6));
            
            % Save route
            V.History.Velo(:,V.t) = V.State(1:3);
            V.History.Pos(:,V.t) = V.State(4:6);
            V.History.Propeller(:,V.t) = [V.Prop.xi; V.Prop.n; V.Prop.Thrust];
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

