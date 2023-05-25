%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% plot_trajectories.m
% 
% Purpose: Plot the desired, actual, dead reckoning, and particle filters paths
%  
% Primary Developer Contact Information:
% Heath Cottrill
% Student
% Statler College of Engineering & Mineral Resources
% Dept. Mechanical and Aerospace Engineering
% West Virginia University (WVU)
% hsc0009@mix.wvu.edu
%
% Development History
% Date              Developer        Comments
% ---------------   -------------    --------------------------------
% May 20, 2023      H. Cottrill      Initial implemention
%

%% Plotting Parameters

plots.markersize=5;
plots.linewidth=1;
plots.colors(1,:) = [0,0,0]; %black
plots.colors(2,:) = [1,0,0]; %red
plots.colors(3,:) = [0,0,1]; %blue
plots.colors(4,:) = [0,1,0]; %green
plots.colors(5,:) = [0.4940 0.1840 0.5560]; %purple
plots.colors(6,:) = [0.8500 0.3250 0.0980]; %purple

%% Trajectories

figure()
clf

hold on
plot3(path(:,1),path(:,2),path(:,3),'LineStyle','--','Color',plots.colors(1,:),'LineWidth',plots.linewidth);
plot3(agent.tpx(1:end),agent.tpy(1:end),agent.tpz(1:end),'LineStyle','-','Color',plots.colors(2,:),'LineWidth',plots.linewidth);
plot3(agent.px(1:end),agent.py(1:end),agent.pz(1:end),'LineStyle',':','Color',plots.colors(3,:),'LineWidth',plots.linewidth);
plot3(mn.px(1:end),mn.py(1:end),mn.pz(1:end),'LineStyle','-.','Color',plots.colors(4,:),'LineWidth',plots.linewidth);
plot3(pr.px(1:end),pr.py(1:end),pr.pz(1:end),'LineStyle','-.','Color',plots.colors(5,:),'LineWidth',plots.linewidth);
plot3(mn.px(1:end),mn.py(1:end),pr.pz(1:end),'LineStyle','-.','Color',plots.colors(6,:),'LineWidth',plots.linewidth);
hold off

xlabel('x (meters)')
ylabel('y (meters)')
zlabel('z (meters)')
legend({'Path','Truth','Dead Reckoning','Terrain-aid Filter','Barometric-aided Filter','Proposed Algorithm'},'Location','best')
view(3)
grid on

%% Trajectory with map

figure()
hold on
plot3(agent.tpx(1:end),agent.tpy(1:end), agent.tpz(1:end), 'LineStyle','--','Color',plots.colors(2,:),'LineWidth',2);
plot3(agent.px(1:end),agent.py(1:end), agent.pz(1:end), 'LineStyle',':','Color',plots.colors(3,:),'LineWidth',2);
plot3(mn.px(1:end),mn.py(1:end),pr.pz(1:end), 'LineStyle','-.','Color',plots.colors(4,:),'LineWidth',2);
surf(Venus_east(1,6450:6467),Venus_north(1,2750:2767),Venus_map(2750:2767,6450:6467)+47500) % Diverse
% surf(Venus_east(1,6450:6467),Venus_north(1,3450:3467),Venus_map(3450:3467,6450:6467)+50000) % Flat
hold off

% c=colorbar('FontSize',12);
% c.Label.String ='Elevation (meters)';
% axis([6450*4.6411e3 6466*4.6411e3 2750*4.6411e3 2766*4.6411e3 -inf inf]);
% axis([6450*4.6411e3 6466*4.6411e3 3450*4.6411e3 3466*4.6411e3 -inf inf]);

xlabel('x [m]');
ylabel('y [m]');
set(gca,'fontsize',12);
legend({'Truth','Dead Reckoning','Proposed Algorithm'},'Location','best');
view(3)

%% Clean Up

clear markersize linewidth