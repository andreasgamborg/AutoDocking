classdef Otter6 < Vessel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
        trim_moment = 0;
        trim_setpoint = 280;
        UseProppeller % boolean for switching between tau or [n xi] as actuator
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
        function O = Otter6(State,Payload)
            %Otter6(State,Payload) Construct an instance of Otter6
            if (nargin < 1),  State = zeros(12,1); end
            if (nargin < 2)
                Payload.m = 0;
                Payload.r = [0;0;0];
            end
            O = O@Vessel(State,Payload);
            O.UseProppeller = 0;
            O.Prop.xi = [0 0]';
            O.Prop.n = [0 0]';
            O.Prop.Thrust = [0 0]';
        end
        function P = getPos(O)
            P = O.State([7 8 12]);
        end
        function x = getMeasurement(O)
            x = O.State();
            x([2 3 4 5 6 9 10 11]) = nan;
            x = x + randn(12,1);
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
            
            nu_r = O.getWaterVelo();
            
            Thrust = Propeller.K*Propeller.R*sin(Propeller.angle)*n.*abs(n) ...
                - Propeller.K*cos(Propeller.angle)*abs(n)*nu_r(1);
            O.Prop.Thrust = Thrust;
            
            Thrust = [Thrust(1) 0 ; 0 Thrust(2)];
            
            tau1 = [cos(xi)'; sin(xi)'; zeros(1,length(xi))] * Thrust;
            tau2 = Smtrx(l1)*tau1(:,1) + Smtrx(l2)*tau1(:,2);
            T = [sum(tau1,2); tau2];
            
        end
        function nu_r = getWaterVelo(O)
            % Calculates the velocity of the vessel relative to the water
            % taking current into account
            u_c = O.Current.v * cos(O.Current.beta - O.State(12));     % current surge velocity
            v_c = O.Current.v * sin(O.Current.beta - O.State(12));     % current sway velocity
            nu_r = O.State(1:6) - [u_c v_c 0 0 0 0]';             % relative velocity vector
        end
        function xdot = getStateDerivative(O)
            % State and current variables
            nu = O.State(1:6);  nu1 = O.State(1:3); nu2 = O.State(4:6);   % velocities
            eta = O.State(7:12);                              % positions
            U = sqrt(nu(1)^2 + nu(2)^2 + nu(3)^2);      % speed
            
            nu_r = O.getWaterVelo();
            
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
            CA  = m2c(MA, nu_r);
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
            Xh = Xu * nu_r(1);
            Yh = Yv * nu_r(2);
            Zh = Zw * nu_r(3);
            Kh = Kp * nu_r(4);
            Mh = Mq * nu_r(5);
            Nh = Nr * (1 + 10 * abs(nu_r(6))) * nu_r(6);
            
            tau_damp = [Xh Yh Zh Kh Mh Nh]';
            
            % Strip theory: cross-flow drag integrals for Yh and Nh
            tau_crossflow = crossFlowDrag(O.L,O.B_pont,T,nu_r);
            
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
            nudot = M \ ( tau + tau_damp + tau_crossflow - C * nu_r - G * eta - g_0);
            etadot = J * nu ;
            xdot = [nudot; etadot];
            
        end
        function step(O,Ts)
            O.t = O.t + 1;
            O.State = O.State + Ts * O.getStateDerivative;
            O.State(10:12) = wrapToPi(O.State(10:12));
            O.trim_moment = O.trim_moment + 0.05 * (O.trim_setpoint - O.trim_moment);
            
            % Save route
            O.History.Velo(:,O.t) = O.State(1:6);
            O.History.Pos(:,O.t) = O.State(7:12);
            O.History.Propeller(:,O.t) = [O.Prop.xi; O.Prop.n; O.Prop.Thrust];
            O.History.Thrust(:,O.t) = O.Thrust;
        end
        function plot(O, T)
            title = 'Course';
            niceplot(O.History.Pos(1,:),O.History.Pos(2,:), [], title, ["--"], ["x [m]", "y [m]"], 'north');
            axis equal
            grid
            set(gca, 'YDir','reverse')
            
            color = 'g-';
            for i = [1:1000:O.t, O.t]
                
                vessel = vesselplot(O.History.Pos(6,i),O.History.Propeller(1:2,i));
                
                if i == O.t
                    color = 'r-';
                end
                
                plot(vessel(1,:)+O.History.Pos(1,i), vessel(2,:)+O.History.Pos(2,i), color, 'LineWidth', 2);
                
                color = 'b-';
            end
            
            
            title = 'Linear Velocities';
            names = ["$u$ surge", "$v$ sway", "$w$ heave"];
            niceplot(T,toKnots(O.History.Velo(1:3,:)), names, title, ["--"], ["time [s]", "[knot]"], 'northeast');
            
            title = 'Orientation';
            names = ["$\phi$ roll", "$\theta$ pitch", "$\psi$ yaw"];
            niceplot(T,rad2deg(O.History.Pos(4:6,:)), names, title, ["--"], ["time [s]", ""], 'south');
            yticks(-180:30:180)
            tl = [180:30:359 0:30:180]+"°"; tl(7) = "N"; tl(10) = "E"; tl([1,13]) = "S"; tl(4) = "W";
            yticklabels(tl)
            
            
            title = 'Angular Velocities';
            names = ["$p$ roll", "$q$ pitch", "$r$ yaw"];
            niceplot(T, rad2deg(O.History.Velo(4:6,:))*60, names, title, ["--"], ["time [s]", "[deg/min]"], 'southeast');
            ytickformat('%.0f°')
            
            if (O.UseProppeller)
                title = 'Propeller velocity';
                names = ["P", "SB"];
                niceplot(T, toRPM(O.History.Propeller(3:4,:)), names, title, ["-"], ["time [s]", "[rpm]"], 'northwest');
                
                title = 'Propeller Thrust';
                names = ["P", "SB"];
                niceplot(T, O.History.Propeller(5:6,:), names, title, ["r-","g--"], ["time [s]", "[N]"], 'southwest');
            else
                title = 'Thrust';
                names = ["$\tau_u$", "$\tau_v$", "$\tau_r$"];
                niceplot(T, O.History.Thrust([1 2 6],:), names, title, ["-"], ["time [s]", "[N]"], 'southwest');
            end
        end
    end
end

