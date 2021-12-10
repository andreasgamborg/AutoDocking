function [CourseIn, CourseOut] = dockingCouse(start,dockfrom,dock)
%dockingCouse(start,dockfrom,dock,time) Compute docking course from point (start) to point (dock) though
%point (dockfrom) as well as the departure course.
%   The function calculates the course from fitting two lines.

% THIS FUNCTION IS NOT FINISHED
% 
%     start = [-20;0];
%     dock = [10;20];
%     dockfrom = dock+[-10;0];
 
    hat = @(x) [-x(2); x(1)];

    approchVector = dockfrom - start;   
    dockingVector = dock - dockfrom;  
    
    approchVector = approchVector/norm(approchVector);
    dockingVector = dockingVector/norm(dockingVector);
    

    k = 4;
    turnStart = dockfrom - k*approchVector;
    turnEnd = dockfrom + k*dockingVector;
    
    R = pinv(hat(approchVector)-hat(dockingVector))*(turnEnd-turnStart);
    CP = turnEnd+R*hat(dockingVector);
    theta = acos(dockingVector'*approchVector/ (norm(approchVector)*norm(dockingVector)));
    
    
    

    w = atan2(turnStart(2),turnStart(1)):-0.05:atan2(turnEnd(2),turnEnd(1));
    w  = [0:0.1:theta]+-pi/2;
    x = R* cos(w);
    y = R* sin(w);

    arc = flip([x;y],2);
    turn = arc+CP;
    
    N = 20;
    step = 1/N;
    approch = [step:step:1; step:step:1].*(turnStart-start) + start;
    docking = [step:step:1; step:step:1].*(dock-turnEnd) + turnEnd;
    approchHeading = ones(1,N) * atan2(approchVector(2),approchVector(1));
    dockingHeading = ones(1,N) * atan2(dockingVector(2),dockingVector(1));
    turnHeading = wrapToPi(atan2(arc(2,:),arc(1,:))-pi/2);



    CourseIn = [  approch    turn     docking;
        approchHeading turnHeading dockingHeading];
    

    undocking = [step:step:1; step:step:1].*(dockfrom-dock) + dock;

    CourseOut =  [  undocking;      dockingHeading];

    end
    
% figure
% plot(CourseIn(1,:),CourseIn(2,:),'ko-','Linewidth',3)
% grid on
% axis equal
% figure
% plot(CourseIn(3,:),'ko-','Linewidth',3)
% grid on
% 
% figure
% plot(CourseOut(1,:),CourseOut(2,:),'ko-','Linewidth',3)
% grid on
% figure
% plot(CourseOut(3,:),'ko-','Linewidth',3)
% grid on


