classdef WayPoint
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        precisionMode
        pos
        accept
    end
    
    methods
        function WP = WayPoint(pos, aDist, aAngle, aSpeed)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if (nargin < 1),  error('Provide location of WayPoint'); end
            if (nargin < 2)  
                WP.setAccept(1,pi,10); 
                WP.precisionMode = false;
            else
                WP.setAccept(aDist, aAngle, aSpeed)
                WP.precisionMode = true;
            end
            WP.pos = pos;
            
        end
        function setAccept(WP, aDist, aAngle, aSpeed)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            WP.accept.distance = aDist;
            WP.accept.angle = aAngle;
            WP.accept.speed = aSpeed;
        end
    end
end

