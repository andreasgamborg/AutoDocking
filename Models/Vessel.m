classdef Vessel < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        State
        Payload
        Current
        Thrust
        Prop
        History
        t = 0;
    end
    properties (Constant)
        g   = 9.81;         % acceleration of gravity (m/s^2)
        rho = 1025;         % density of water (kg/m^3)
    end
    methods
        function V = Vessel(State,Payload)
            %Ferry3(State,Payload) Construct an instance of Ferry3
            %   Detailed explanation goes here
            V.State = State;
            V.Payload = Payload;
            V.Current.v = 0;
            V.Current.beta = 0;
        end
        function setCurrent(V,v,beta)
            %updateCurrent(obj,inputArg) Summary of this method goes here
            %   Detailed explanation goes here
            V.Current.v = v;
            V.Current.beta = beta;
        end
        function Vessel = setProp(Vessel,n,xi)
            %updateCurrent(obj,inputArg) Summary of this method goes here
            %   Detailed explanation goes here
            Vessel.Prop.n = n;
            Vessel.Prop.xi = xi;
        end
        function setPayload(V,m,r)
            %setPayload(V,m,r) Set a new payload for the Vessel
            %   Detailed explanation goes here
            V.Payload.m = m;
            V.Payload.r = r;
        end
    end
    methods (Abstract)
        updateThrust(V)
        getStateDerivative(V)
        step(V,Ts)
    end
end


