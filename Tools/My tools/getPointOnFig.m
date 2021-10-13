function P = getPointOnFig(n,path)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    %data = O6.History.Pos(1:2,:);
    path = 'Rute luftfoto.png';
    pixelsPerMeter = 3;

    background = imread(path);
    [N,M,~] = size(background);

    close all
    figure;
    imagesc([-M/pixelsPerMeter M/pixelsPerMeter], [-N/pixelsPerMeter N/pixelsPerMeter], background); hold on

    % plot(data(2,:),-data(1,:))
    axis equal

    P = ginput(n);
    P = [P' ; zeros(1,n)];
end

