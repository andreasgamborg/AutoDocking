function [CourseIn, CourseOut] = dockingCouse(start,dockfrom,dock)
%dockingCouse(start,dockfrom,dock,time) Compute docking course from point (start) to point (dock) though
%point (dockfrom) as well as the departure course.
%   The function calculates the course from fitting two lines.

    % start = [1;2];
    % dock = [20;18];
    % dockfrom = dock+[-6;0];

    approchVector = dockfrom-start;
    dockingVector = dock - dockfrom;

    N = 20;
    step = 1/N;
    approch = [step:step:1; step:step:1].*approchVector + start;
    docking = [step:step:1; step:step:1].*dockingVector + dockfrom;


    approchHeading = ones(1,N) * atan2(approchVector(2),approchVector(1));
    dockingHeading = ones(1,N) * atan2(dockingVector(2),dockingVector(1));


    CourseIn = [  approch         docking
        approchHeading  dockingHeading];

    CourseOut =  [  flip(docking,2)-step*dockingVector
        dockingHeading];

end
%
% figure
% plot(CourseIn(1,:),CourseIn(2,:),'ko-','Linewidth',3)
% grid on
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


