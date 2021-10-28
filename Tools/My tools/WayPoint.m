classdef WayPoint < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        precisionMode
        pos
        heading
        accept
    end
    
    methods
        function WP = WayPoint(pos, heading, aDist, aAngle)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if (nargin < 1),  error('Provide location of WayPoint'); end
            if (nargin < 2)
                heading = 0;
                WP.setAccept(1,pi); 
            end
            if (nargin < 3)  
                WP.setAccept(0.5,pi/36); % 50cm  5deg 
                WP.precisionMode = false;
            else
                WP.setAccept(aDist, aAngle)
                WP.precisionMode = true;
            end
            WP.pos = pos;
            WP.heading = heading;
        end
        function setAccept(WP, aDist, aAngle)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            WP.accept.distance = aDist;
            WP.accept.angle = aAngle;
        end
    end
end

