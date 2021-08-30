classdef Ferry3 < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        State
        Payload
        Current
        Prop
        History
        t = 0;
    end
    properties (Constant)
        g   = 9.81;         % acceleration of gravity (m/s^2)
        rho = 1025;         % density of water (kg/m^3)
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
        function Vessel = Ferry3(State,Payload)
            %Ferry3(State,Payload) Construct an instance of Ferry3
            %   Detailed explanation goes here
            if nargin > 0
                Vessel.State = State;
            else
                Vessel.State = zeros(6,1);
            end
            
            if nargin > 1
                Vessel.Payload = Payload;
            else
                Vessel.Payload.m = 0;
                Vessel.Payload.r = [0;0;0];
            end
            Vessel.Current.v = 0;
            Vessel.Current.beta = 0;
            Vessel.Prop.n = [0;0];
            Vessel.Prop.xi = [0;0];
        end
        
        function Vessel = updateCurrent(Vessel,v,beta)
            %updateCurrent(obj,inputArg) Summary of this method goes here
            %   Detailed explanation goes here
            Vessel.Current.v = v;
            Vessel.Current.beta = beta;
        end
        function Vessel = updateProp(Vessel,n,xi)
            %updateCurrent(obj,inputArg) Summary of this method goes here
            %   Detailed explanation goes here
            Vessel.Prop.n = n;
            Vessel.Prop.xi = xi;
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
        function xdot = dferry3(Vessel)
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
            dx = V.dferry3();
            V.State = V.State + Ts * dx;
            V.State(6) = wrapToPi(V.State(6));
            
            % Save route
            V.History.Velo(:,V.t) = V.State(1:3);
            V.History.Pos(:,V.t) = V.State(4:6);
            V.History.Propeller(:,V.t) = [V.Prop.xi; V.Prop.n; V.Prop.Thrust];
        end
    end
end

