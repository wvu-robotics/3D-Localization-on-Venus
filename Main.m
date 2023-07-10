% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% Main.m
% 
% Purpose: Simulatation of "aerobot" localization around Venus
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

%% Cleaning

% Clean
clear
close
clc

% Display Progress
disp('Cleaning...')
tic

% Specify Seed
rng('default')

%% Parameters

% Display current progress
disp('Parameters...')

% Time
time = 10*60; % Total time of simulation in seconds
dt = 0.25; % Time step for dead reckoning
dt_pf = 0.25; % Time step for particle filter

% Pose
attitude = [0;0;0]; % Initial orientation defined by roll, pitch, and yaw [deg]
velocity = [0;0;0]; % Initial velocity defined in the body frame [m/s]
position = [0;0;60000]; % Initial position desfined by latitude, longitude, [deg] and height [m]

% Accelerometer
bias_static_accel = 0.7; % Static bias of acceleration [mg]
bias_dynamic_accel = 0.2; % Dynamic bias of acceleration [mg]
random_noise_accel = 0.17; % Random noise of acceleration [m/sec/sqrt(hr)]

% Gyroscope
bias_static_gyro = 20; % Static bias of angular velocity [deg/hr]
bias_dynamic_gyro = 1; % Dynamic bias of angular velocity [deg/hr]
random_noise_gyro = 0.25; % Random noise of angular velocity [deg/sqrt(hr)]

% Barometer
bias_static_barometer = 0.03; % Static bias of angular velocity [hPa]
bias_dynamic_barometer = 0.1; % Dynamic bias of angular velocity [hPa/year]
random_noise_barometer = 0.07; % Random noise of pressure measure [hPa]

% Altimeter
bias_static_altimeter = 0; % Static bias of angular velocity [m]
bias_dynamic_alimeter = 0; % Dynamic bias of angular velocity [m]
random_noise_altimeter = 0.3; % Random noise of height measure [m]

% Particle Filter
npf = 1000; % Number of particles
P0(7,7) = 10*(1/6051800); % Initial dispersion of particles in latitude direction [m]
P0(8,8) = 10*(1/6051800); % Initial dispersion of particles in longitude direction [m]
P0(9,9) = 10; % Initial dispersion of particles in height [m]
sigma_lat = 10*(1/6051800); % Roughening of particles in latitude direction [m]
sigma_lon = 10*(1/6051800); % Roughening of particles in latitude direction [m]
sigma_h = 5; % Roughening of particles in height direction [m]
r = 10; % Adding of jitter to the process model


%% Initialization

% Display current progress
disp('Initialization...')

% Time
t = 0:dt:time;
L = time/dt+1;

% Preallocation
ins_att = zeros(3,L);
ins_vel = zeros(3,L);
ins_pos = zeros(3,L);
ins_pos_xyz = zeros(3,L);
ins_pos_ned = zeros(3,L);
omega_b_ib = zeros(3,L);
f_ib_b = zeros(3,L);
E = zeros(9,L);
Cov = zeros(9,L);

% Pose
ins_att(:,1) = deg2rad(attitude); % Convert to rad
ins_vel(:,1) = angle2dcm(ins_att(1,1),ins_att(2,1),ins_att(3,1),'XYZ')*velocity; % Convert to Local-Navigation-Frame
ins_pos(:,1) = [deg2rad(position(1));deg2rad(position(2));position(3)]; % Convert to rad
ins_pos_xyz(:,1) = llh2xyz(ins_pos(:,1)); % Convert from geographic to ECEF cartesian coordinates
ins_pos_ned(:,1) = xyz2ned(ins_pos(:,1),ins_pos_xyz(:,1),ins_pos_xyz(:,1));
x0 = [ins_att(:,1);ins_vel(:,1);ins_pos(:,1)];

% Accelerometer
bias_static_accel = bias_static_accel*8.858/1000; % Convert to [m/s^2]
bias_dynamic_accel = bias_dynamic_accel*8.858/1000; % Convert to [m/s^2]
random_noise_accel = random_noise_accel/sqrt(3600)/sqrt(dt); % Convert to [m/s^2]

% Gyroscope
bias_static_gyro = bias_static_gyro*(pi/180)*(1/3600); % Convert to [rad/s]
bias_dynamic_gyro = bias_dynamic_gyro*(pi/180)*(1/3600); % Convert to [rad/s]
random_noise_gyro = random_noise_gyro*(pi/180)*(1/sqrt(3600))/sqrt(dt); % Convert to [rad/s]

% Barometer
bias_static_barometer = bias_static_barometer*100; % Convert to [Pa]
bias_dynamic_barometer = bias_dynamic_barometer*100*(1/31536000)*dt_pf; % Convert to [Pa]
random_noise_barometer = random_noise_barometer*100; % Convert to [Pa]

% Accelerometer Error
b_as = normrnd(0,bias_static_accel,[1,L]);
b_ad = zeros(1,L);
for j = 2:L
    b_ad(j) = b_ad(j-1)+normrnd(0,bias_dynamic_accel/sqrt(1/dt));
end
w_a = normrnd(0,random_noise_accel,[1,L]);
error_accel = b_as+b_ad+w_a;

% Gyroscope Error
b_gs = normrnd(0,bias_static_gyro,[1,L]);
b_gd = zeros(1,L);
for j = 2:L
    b_gd(j) = b_gd(j-1)+normrnd(0,bias_dynamic_gyro/sqrt(1/dt));
end
w_g = normrnd(0,random_noise_gyro,[1,L]);
error_gyro = b_gs+b_gd+w_g;

% Barometer Error
b_bs = normrnd(0,bias_static_barometer,[1,L]);
b_bd = zeros(1,L);
for j = 2:L
    b_bd(j) = b_bd(j-1)+normrnd(0,bias_dynamic_barometer/sqrt(1/dt));
end
w_b = normrnd(0,random_noise_barometer,[1,L]);
error_barometer = b_bs+b_bd+w_b;

% Altimeter Error
error_altimeter = normrnd(0,random_noise_altimeter,[1,L]);

% Venus Map
load('Venus_map.mat')
longitude = Venus_xm_lon;
latitude = Venus_ym_lat';
height = Venus_map';
map = griddedInterpolant({longitude,latitude},height); %,'nearest'

% Particle Filter
x = repmat(x0,1,npf) + P0 * randn(length(P0),npf); % State Matrix
w = ones(npf,1)/npf; % Particle Weight
q = ones(npf,1)/npf; % Particle Likelihood

%% Simulation

% Iterate for truth [1], estimate [2], and particle filter [3]
for i = 1:3

    % Display Progress
    if i == 1
        disp('Truth Simulation...')
    elseif i == 2
        disp('Noisy Simulation...')
    elseif i == 3
        disp('PF Simulation...')
    end

    % Iterate for all time steps less the first
    for j = 2:L

        % If truth
        if i == 1

            % Guidance Model
            [omega_b_ib(:,j),f_ib_b(:,j)] = Guidance_Model(ins_att(:,j-1),ins_vel(:,j-1),ins_pos(:,j-1),ins_pos_xyz(:,j-1),dt);

            % Dead Reckoning
            [ins_att_plus,ins_vel_plus,ins_pos_plus,ins_pos_plus_xyz] = Dead_Reckoning(ins_att(:,j-1),ins_vel(:,j-1),ins_pos(:,j-1),omega_b_ib(:,j),f_ib_b(:,j)*dt,dt);

            % Update States
            ins_att(:,j) = ins_att_plus;
            ins_vel(:,j) = ins_vel_plus;
            ins_pos(:,j) = ins_pos_plus;
            ins_pos_xyz(:,j) = ins_pos_plus_xyz;
            ins_pos_ned(:,j) = xyz2ned(ins_pos_plus,ins_pos_plus_xyz,ins_pos_xyz(:,1));

            % Display Progress
            if(mod(j,0.1*(L-1))==0)
                progress = j/(L-1)*100;
                fprintf('%.0f%% \n',progress);
            end
        
        % If estimate
        elseif i == 2

            % Add error
            omega_b_ib(:,j) = omega_b_ib(:,j) + error_gyro(j); 
            f_ib_b(:,j) = f_ib_b(:,j) + error_accel(j);

            % Dead Reckoning
            [ins_att_plus,ins_vel_plus,ins_pos_plus,ins_pos_plus_xyz] = Dead_Reckoning(ins_att(:,j-1),ins_vel(:,j-1),ins_pos(:,j-1),omega_b_ib(:,j),f_ib_b(:,j)*dt,dt);

            % Update States
            ins_att(:,j) = ins_att_plus;
            ins_vel(:,j) = ins_vel_plus;
            ins_pos(:,j) = ins_pos_plus;
            ins_pos_xyz(:,j) = ins_pos_plus_xyz;
            ins_pos_ned(:,j) = xyz2ned(ins_pos_plus,ins_pos_plus_xyz,ins_pos_xyz(:,1));

            % Display Progress
            if(mod(j,0.1*(L-1))==0)
                progress = j/(L-1)*100;
                fprintf('%.0f%% \n',progress);
            end
        
        % If particle filter
        elseif i == 3

            % Dead Reckoning of each particle
            orient = x(1:3,:);
            vel = x(4:6,:);
            pos = x(7:9,:);
            gyro = omega_b_ib(:,j);
            accel = f_ib_b(:,j);
            parfor k = 1:npf
                [ins_att_plus,ins_vel_plus,ins_pos_plus] = Dead_Reckoning(orient(:,k),vel(:,k),pos(:,k),gyro,accel*dt,dt);
                x(:,k) = [ins_att_plus;ins_vel_plus;ins_pos_plus];
            end
                
            % If update is available
            if mod((j-1)*dt,dt_pf) == 0
    
                % Measurements with noise
                alt = map(rad2deg(truth.pos(2,j)),rad2deg(truth.pos(1,j))) + error_altimeter(j);
                pres = air_pressure(truth.pos(3,j)) + error_barometer(j);
                yNow = repmat([alt pres],npf,1);
    
                % Estimated measurements of each particle
                yhat(:,1) = map(rad2deg(x(8,:)'),rad2deg(x(7,:)'));
                yhat(:,2) = air_pressure(x(9,:))';
    
                % Likelihood of each particle
                q = normpdf(yNow(:,1),yhat(:,1),r*random_noise_altimeter).*normpdf(yNow(:,2),yhat(:,2),r*random_noise_barometer);
    
                % Multiply previous weight
                q = q .* w; 

                % Normalize the weights
                if sum(q) ~= 0
                    q = q ./ sum(q);
                end

                % Copy the weights
                w = q; 
    
                % Effective particle count
                if sum(q) == 0
                    break
                else
                    N_eff = 1/sum(q.^2);
                end
    
                % If count below threshold...
                if N_eff < 0.5*npf

                    % Resample
                    [x,w] = MSVResampling(x,w);
                    % [x,w] = SystematicResampling(x,w);
                    % [x,w] = ResidualResampling(x,w);
                    
                end
            end

            % State predicted by Particle Filter
            E(:,j) = [sum(q.*x(1,:)') sum(q.*x(2,:)') sum(q.*x(3,:)') sum(q.*x(4,:)') sum(q.*x(5,:)') sum(q.*x(6,:)') sum(q.*x(7,:)') sum(q.*x(8,:)') sum(q.*x(9,:)')]';
            Cov(:,j) = [sum(q.*(x(1,:)-E(1,j))*(x(1,:)-E(1,j))') sum(q.*(x(2,:)-E(2,j))*(x(2,:)-E(2,j))') sum(q.*(x(3,:)-E(3,j))*(x(3,:)-E(3,j))') sum(q.*(x(4,:)-E(4,j))*(x(4,:)-E(4,j))') sum(q.*(x(5,:)-E(5,j))*(x(5,:)-E(5,j))') sum(q.*(x(6,:)-E(6,j))*(x(6,:)-E(6,j))') sum(q.*(x(7,:)-E(7,j))*(x(7,:)-E(7,j))') sum(q.*(x(8,:)-E(8,j))*(x(8,:)-E(8,j))') sum(q.*(x(9,:)-E(9,j))*(x(9,:)-E(9,j))')]';
            ins_att(:,j) = E(1:3,j);
            ins_vel(:,j) = E(4:6,j);
            ins_pos(:,j) = E(7:9,j);
            ins_pos_xyz(:,j) = llh2xyz(ins_pos(:,j));
            ins_pos_ned(:,j) = xyz2ned(ins_pos(:,j),ins_pos_xyz(:,j),ins_pos_xyz(:,1));

            % Roughening
            x(7,:) = x(7,:)+normrnd(0,sigma_lat,[1,npf]);
            x(8,:) = x(8,:)+normrnd(0,sigma_lon,[1,npf]);
            x(9,:) = x(9,:)+normrnd(0,sigma_h,[1,npf]);

            % Display Progress
            if(mod(j,0.01*(L-1))==0)
                progress = j/(L-1)*100;
                fprintf('%.0f%% \n',progress);
            end
        end
    end

    % If truth
    if i == 1

        % Save data
        truth.att = ins_att;
        truth.vel = ins_vel;
        truth.pos = ins_pos;
        truth.pos_xyz = ins_pos_xyz;
        truth.pos_ned = ins_pos_ned;
        truth.omega_b_ib = omega_b_ib;
        truth.f_ib_b = f_ib_b;
        truth.state = [ins_att;ins_vel;ins_pos];

        % Clear data
        ins_att(:,2:end) = zeros(size(ins_att(:,2:end)));
        ins_vel(:,2:end) = zeros(size(ins_vel(:,2:end)));
        ins_pos(:,2:end) = zeros(size(ins_pos(:,2:end)));
        ins_pos_xyz(:,2:end) = zeros(size(ins_pos_xyz(:,2:end)));
        ins_pos_ned(:,2:end) = zeros(size(ins_pos_ned(:,2:end)));

    % If estimate 
    elseif i == 2

        % Save Data
        estimate.att = ins_att;
        estimate.vel = ins_vel;
        estimate.pos = ins_pos;
        estimate.pos_xyz = ins_pos_xyz;
        estimate.pos_ned = ins_pos_ned;
        estimate.omega_b_ib = omega_b_ib;
        estimate.f_ib_b = f_ib_b;
        estimate.state = [ins_att;ins_vel;ins_pos];

        % Clear data
        ins_att(:,2:end) = zeros(size(ins_att(:,2:end)));
        ins_vel(:,2:end) = zeros(size(ins_vel(:,2:end)));
        ins_pos(:,2:end) = zeros(size(ins_pos(:,2:end)));
        ins_pos_xyz(:,2:end) = zeros(size(ins_pos_xyz(:,2:end)));
        ins_pos_ned(:,2:end) = zeros(size(ins_pos_ned(:,2:end)));

    % If particle filter
    elseif i == 3

        % Save Data
        pf.att = ins_att;
        pf.vel = ins_vel;
        pf.pos = ins_pos;
        pf.pos_xyz = ins_pos_xyz;
        pf.pos_ned = ins_pos_ned; 
        pf.state = [ins_att;ins_vel;ins_pos];
        pf.E = E;
        pf.Cov = Cov;
    end
end

%% Plots

% Display Progress
disp('Figure Plotting...')

% Plot paths in LLH coordinates
figure(1)

hold on
plot3(rad2deg(truth.pos(1,:)),rad2deg(truth.pos(2,:)),truth.pos(3,:),'k',LineWidth=2)
plot3(rad2deg(estimate.pos(1,:)),rad2deg(estimate.pos(2,:)),estimate.pos(3,:),'r',LineWidth=2)
plot3(rad2deg(pf.pos(1,1:j-1)),rad2deg(pf.pos(2,1:j-1)),pf.pos(3,1:j-1),'b',LineWidth=2)
title('Position');
xlabel('Latitude [deg]');
ylabel('Longitude [deg]');
zlabel('Height [m]');
legend({'Truth','Estimate','PF'},Location='eastoutside')
view(3)
grid on
hold off

% Plot RSME errors in LLH coordinates
figure(2)

subplot(311)
hold on
plot(t/60,rad2deg(rmse(truth.pos(1,:),estimate.pos(1,:),1)),'k',LineWidth=2); 
plot(t(1,1:j-1)/60,rad2deg(rmse(truth.pos(1,1:j-1),pf.pos(1,1:j-1),1)),'r',LineWidth=2); 
title('Latitude');
xlabel('Time [min]');
ylabel('RMSE [deg]');
legend({'Dead Reckoning','Particle Filter'})
grid on

subplot(312)
hold on
plot(t/60,rad2deg(rmse(truth.pos(2,:),estimate.pos(2,:),1)),'k',LineWidth=2); 
plot(t(1,1:j-1)/60,rad2deg(rmse(truth.pos(2,1:j-1),pf.pos(2,1:j-1),1)),'r',LineWidth=2); 
title('Longitude');
xlabel('Time [min]');
ylabel('RMSE [deg]');
legend({'Dead Reckoning','Particle Filter'})
grid on

subplot(313)
hold on
plot(t/60,rmse(truth.pos(3,:),estimate.pos(3,:),1),'k',LineWidth=2); 
plot(t(1,1:j-1)/60,rmse(truth.pos(3,1:j-1),pf.pos(3,1:j-1),1),'r',LineWidth=2);
title('Height');
xlabel('Time [min]');
ylabel('RMSE [m]');
legend({'Dead Reckoning','Particle Filter'})
grid on

% Plot paths in NED coordinates
figure(3)

hold on
plot3(truth.pos_ned(1,:),truth.pos_ned(2,:),truth.pos_ned(3,:),'k',LineWidth=2)
plot3(estimate.pos_ned(1,:),estimate.pos_ned(2,:),estimate.pos_ned(3,:),'r',LineWidth=2)
plot3(pf.pos_ned(1,1:j-1),pf.pos_ned(2,1:j-1),pf.pos_ned(3,1:j-1),'b',LineWidth=2)
title('Position');
xlabel('North [m]');
ylabel('East [m]');
zlabel('Down [m]');
legend({'Truth','Estimate','PF'},Location='eastoutside')
view(3)
grid on
hold off

% Plot RSME error in NED coordinates
figure(4)

subplot(311)
hold on
plot(t/60,rmse(truth.pos_ned(1,:),estimate.pos_ned(1,:),1),'k',LineWidth=2); 
plot(t(1,1:j-1)/60,rmse(truth.pos_ned(1,1:j-1),pf.pos_ned(1,1:j-1),1),'r',LineWidth=2); 
title('North Direction');
xlabel('Time [min]');
ylabel('RMSE [m]');
legend({'Dead Reckoning','Particle Filter'})
grid on

subplot(312)
hold on
plot(t/60,rmse(truth.pos_ned(2,:),estimate.pos_ned(2,:),1),'k',LineWidth=2); 
plot(t(1,1:j-1)/60,rmse(truth.pos_ned(2,1:j-1),pf.pos_ned(2,1:j-1),1),'r',LineWidth=2); 
title('East Direction');
xlabel('Time [min]');
ylabel('RMSE [m]');
legend({'Dead Reckoning','Particle Filter'})
grid on

subplot(313)
hold on
plot(t/60,rmse(truth.pos_ned(3,:),estimate.pos_ned(3,:),1),'k',LineWidth=2); 
plot(t(1,1:j-1)/60,rmse(truth.pos_ned(3,1:j-1),pf.pos_ned(3,1:j-1),1),'r',LineWidth=2);
title('Down Direction');
xlabel('Time [min]');
ylabel('RMSE [m]');
legend({'Dead Reckoning','Particle Filter'})
grid on

% Plot paths over terrain map
figure(5)

hold on
view(3)
plot3(rad2deg(truth.pos(1,:)),rad2deg(truth.pos(2,:)),truth.pos(3,:),'k',LineWidth=2)
plot3(rad2deg(estimate.pos(1,:)),rad2deg(estimate.pos(2,:)),estimate.pos(3,:),'r',LineWidth=2)
plot3(rad2deg(pf.pos(1,1:j-1)),rad2deg(pf.pos(2,1:j-1)),pf.pos(3,1:j-1),'b',LineWidth=2)
axis padded
info = axis;

[~, ii] = min(abs(Venus_ym_lat-info(1)));
[~, y_min] = ind2sub(size(Venus_ym_lat), ii);
[~, ii] = min(abs(Venus_ym_lat-info(2)));
[~, y_max] = ind2sub(size(Venus_ym_lat), ii);
lat = Venus_ym_lat(1,y_min-1:y_max+1);

[~, ii] = min(abs(Venus_xm_lon-info(3)));
[~, x_min] = ind2sub(size(Venus_xm_lon), ii);
[~, ii] = min(abs(Venus_xm_lon-info(4)));
[~, x_max] = ind2sub(size(Venus_xm_lon), ii);
lon = Venus_xm_lon(1,x_min-1:x_max+1);

surface = Venus_map(y_min-1:y_max+1,x_min-1:x_max+1);
surf(lat,lon,surface') 

title('Position');
xlabel('Latitude [deg]');
ylabel('Longitude [deg]');
zlabel('Height [m]');
legend({'Truth','Estimate','PF'},Location='southoutside')
colorbar('FontSize',12);
set(gca,'fontsize',12);
grid on

%% Time

% Display time elapsed
toc
