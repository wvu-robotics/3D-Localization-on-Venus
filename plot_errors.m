%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% plot_error.m
% 
% Purpose: Plot the 3D position errors of dead reckonng and particle filters
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
plots.colors(1,:) = [1,0,0]; %red
plots.colors(2,:) = [0,0,1]; %blue
plots.colors(3,:) = [0,0,0]; %black
plots.colors(4,:) = [0,1,0]; %green
plots.colors(5,:) = [0.4940 0.1840 0.5560]; %purple

%% Position Error

figure()
clf

subplot(311)
plots.time = simulation.Ts*1:length(errors.mn.px);

hold on
plots.err_posx_mn = plot(plots.time/600,errors.mn.px,'LineStyle','-.','Color',plots.colors(1,:),'LineWidth',plots.linewidth); 
plots.err_posx_pr = plot(plots.time/600,errors.pr.px,'LineStyle','-.','Color',plots.colors(2,:),'LineWidth',plots.linewidth); 
plots.err_posx_dr = plot(plots.time/600,errors.dr.px,'LineStyle',':','Color',plots.colors(3,:),'LineWidth',plots.linewidth);
hold off

title('X Position Error');
xlabel('Time [min]');
ylabel('Error [m]');
grid on;
legend([plots.err_posx_dr,plots.err_posx_mn,plots.err_posx_pr],'Terrain-Aided Filtering','Barometric-Based Filtering','Dead Reckoning','Location','eastoutside');

subplot(312)
plots.time = simulation.Ts*1:length(errors.mn.py);

hold on
plots.err_posy_mn = plot(plots.time/600,errors.mn.py,'LineStyle','-.','Color',plots.colors(1,:),'LineWidth',plots.linewidth);
plots.err_posy_pr = plot(plots.time/600,errors.pr.py,'LineStyle','-.','Color',plots.colors(2,:),'LineWidth',plots.linewidth);
plots.err_posy_dr = plot(plots.time/600,errors.dr.py,'LineStyle',':','Color',plots.colors(3,:),'LineWidth',plots.linewidth);
hold off

title('Y Position Error');
xlabel('Time [min]');
ylabel('Error [m]');
grid on;
legend([plots.err_posy_dr,plots.err_posy_mn,plots.err_posy_pr],'Terrain-Aided Filtering','Barometric-Based Filtering','Dead Reckoning','Location','eastoutside');

subplot(313)
plots.time = simulation.Ts*1:length(errors.mn.pz);

hold on
plots.err_posz_mn=plot(plots.time/600,errors.mn.pz,'LineStyle','-.','Color',plots.colors(1,:),'LineWidth',plots.linewidth);
plots.err_posz_pr=plot(plots.time/600,errors.pr.pz,'LineStyle','-.','Color',plots.colors(2,:),'LineWidth',plots.linewidth);
plots.err_posz_dr=plot(plots.time/600,errors.dr.pz,'LineStyle',':','Color',plots.colors(3,:),'LineWidth',plots.linewidth);
hold off

title('Z Position Error');
xlabel('Time [min]');
ylabel('Error [m]');
grid on;
legend([plots.err_posz_dr,plots.err_posz_mn,plots.err_posz_pr],'Terrain-Aided Filtering','Barometric-Based Filtering','Dead Reckoning','Location','eastoutside');

%% TOTAL RESIDUAL

figure()
plots.time = simulation.Ts*1:length(errors.resid_pf1);
hold on
plots.resid_PF1=plot(plots.time/600,errors.resid_pf1,'LineStyle','-','Color',plots.colors(2,:),'LineWidth',plots.linewidth);
plots.resid_PF2=plot(plots.time/600,errors.resid_pf2,'LineStyle','-','Color',plots.colors(3,:),'LineWidth',plots.linewidth);

title('Residual of Particle filters');
ylabel('Residual [m]');
xlabel('Time [min]');
grid on;
legend([plots.resid_PF1,plots.resid_PF2],'Terrain-Aided Filtering','Barometric-Based Filtering');

%% Clean Up

clear markersize linewidth i