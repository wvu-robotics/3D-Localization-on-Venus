%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% plot_maps.m
% 
% Purpose: Plot terrain and pressure map of Venus
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
% July 10, 2023     H. Cottrill      Initial implemention
% 

%% Initialization

clear
close all
load Venus_map.mat

%% Terrain Map [LLH]

figure()
hold on
grid on

mesh(Venus_xm_lon,Venus_ym_lat,Venus_map)
c=colorbar('FontSize',12);
c.Label.String ='Elevation [m]';
axis([-5 5 -5 5 -inf inf]);
xlabel('Longitude [deg]');
ylabel('Latitude [deg]');
set(gca,'fontsize',12);
view(0,90)

%% Terrain Map [XYZ]

figure()
hold on
grid on

mesh(Venus_east,Venus_north,Venus_map)
plot3(6458*4.6411e3,3458*4.6411e3,1.25e4,'square','MarkerSize',10,'MarkerEdgeColor','black','MarkerFaceColor',[1 0 0])
plot3(6458*4.6411e3,2750*4.6411e3,1.25e4,'square','MarkerSize',10,'MarkerEdgeColor','black','MarkerFaceColor',[0 1 0])
c=colorbar('FontSize',12);
c.Label.String ='Elevation [m]';
axis([0 3.801491344990000e+07 0 1.900513619550000e+07 -inf inf])
xlabel('x [m]')
ylabel('y [m]')
set(gca,'fontsize',12)
view(0,90)

%% Terrain Map [Emphasis 1]

figure()
hold on
grid on

surf(Venus_east(1,6450:6467),Venus_north(1,2750:2767),Venus_map(2750:2767,6450:6467))
% c=colorbar('FontSize',12);
% c.Label.String ='Elevation [m]';
axis([6450*4.6411e3 6466*4.6411e3 2750*4.6411e3 2766*4.6411e3 0 2400]);
xlabel('x [m]')
ylabel('y [m]')
set(gca,'fontsize',12)
view(3)

%% Terrain Map [Emphasis 2]

figure()
hold on
grid on

surf(Venus_east(1,6450:6467),Venus_north(1,3450:3467),Venus_map(3450:3467,6450:6467))
% c=colorbar('FontSize',12);
% c.Label.String ='Elevation [m]';
axis([6450*4.6411e3 6466*4.6411e3 3450*4.6411e3 3466*4.6411e3 0 2400]);
xlabel('x [m]')
ylabel('y [m]')
set(gca,'fontsize',12)
view(3)

%% Pressure Map

altitude = 50000:75000;
pressure = 1000*air_pressure(altitude);

figure()
hold on
grid on

area([0 pressure],[75000 altitude])
xlabel('Pressure [Pa]')
ylabel('Altitude [m]')
ylim([50000 75000])
