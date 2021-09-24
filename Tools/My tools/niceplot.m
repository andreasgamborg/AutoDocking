function fig = niceplot(time,series, names, title_in, style, axisnames, position)
%niceplot wrapper function for creating plots
%   niceplot(time,series, names, title_in, style, axisnames, position)

Nseries = size(series,1);
fontsize = 26;
Monitor = 2;

if nargin<3
  names = "series "+string(1:1:Nseries);
end

if nargin<4 || isempty(title_in)
    title0 = "Nice plot";
else
    title0 = title_in;
end

if nargin<5
  style = ['-'];
end
if nargin<6
  axisnames = ["x","y"];
end
if nargin<7
  position = [0 0];
end

Nstyles = size(style,2);

if mod(Nseries,Nstyles) ~= 0
    error("The number of series must be devideable by the number of styles");
else
    style_cutsize = Nseries/Nstyles;
end

if isempty(names)
elseif size(names,2) ~= Nseries
    error("Incorrect number of names for legend");
end 

MonitorPositions = get(0, 'MonitorPositions');
if (Monitor > size(MonitorPositions,1)), warning("You dont have "+Monitor+" monitors, using primary"), Monitor = size(MonitorPositions,1); end
MonitorPositions(:,3) = MonitorPositions(:,3)/3;
MonitorPositions(:,4) = MonitorPositions(:,4)/2;

fig = figure('Name',title0,'DefaultAxesFontSize',fontsize,'OuterPosition',MonitorPositions(Monitor,:));

if ~isempty(title_in)
    %title(title0, 'Interpreter','latex');
end
xlabel(axisnames(1), 'Interpreter','latex');
ylabel(axisnames(2), 'Interpreter','latex')

hold on
for it = 1:1:Nstyles
    cut_start = (it-1)*style_cutsize+1;
    cut_end = it*style_cutsize;
    plot(time,series(cut_start:cut_end,:),style(it),'Linewidth',3);
end
% hold off

if ~isempty(names)
    lgd = legend(names,'Interpreter', 'LaTeX');
    lgd.NumColumns = ceil(Nseries/4);
    % lgd.NumColumns = 2;
    lgd.FontSize = fontsize;
end

movegui(fig,position);

end

