% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% Venus_Localization_Simulation_Single.m
% 
% Purpose: Simulatation of "aerobot" localization around Venus using PF
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
% Dec. 4, 2023      H. Cottrill      Initial implemention
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
rng(83)

%% Parameters

% Display current progress
disp('Parameters...')

% Time
time = 12*60*60; % Total time of simulation in seconds
dt = 0.1; % Time step for INS
dt_pf = 0.1; % Time step for particle filter

% Accelerometer
bias_static_accel = 0.3; % Static bias of acceleration [mg]
bias_dynamic_accel = 0.03; % Dynamic bias of acceleration [mg]
random_noise_accel = 0.02; % Random noise of acceleration [m/sec/sqrt(hr)]

% Gyroscope
bias_static_gyro = 0.15; % Static bias of angular velocity [deg/hr]
bias_dynamic_gyro = 0.015; % Dynamic bias of angular velocity [deg/hr]
random_noise_gyro = 0.01; % Random noise of angular velocity [deg/sqrt(hr)]

% Process Noise
sigma_roll = deg2rad(1); % Process noise of roll angle [deg]
sigma_pitch = deg2rad(1); % Process noise of pitch angle [deg]
sigma_yaw = deg2rad(1); % Process noise of yaw angle [deg]
sigma_u = 1; % Process noise of velocity in north direction [m/s]
sigma_v = 1; % Process noise of velocity in east direction [m/s]
sigma_w = 1; % Process noise of velocity in down direction [m/s]
sigma_lat = 10*(1/6051800); % Process noise of position in latitude direction [m]
sigma_lon = 10*(1/6051800); % Process noise of position in longitude direction [m]
sigma_h = 10; % Process noise of position in height direction [m]
sigma_gyro_bias = 1*(pi/180)*(1/3600); % Process noise of gyroscope bias [rad/s]
sigma_accel_bias = 1*(8.858/1000); % Process noise of accelerometer bias [m/s^2]
r = 25; % Adding of roughening/jitter to the model

% Measurement Noise
sigma_a = 5; % Measurement noise of height measure [m]
sigma_b = 40; % Measurement noise of pressure measure [Pa]

% Initial Covariance
cov_roll = deg2rad(0); % Covariance of roll angle [deg]
cov_pitch = deg2rad(0); % Covariance of pitch angle [deg]
cov_yaw = deg2rad(0); % Covariance of yaw angle [deg]
cov_u = 0; % Covariance of velocity in north direction [m/s]
cov_v = 0; % Covariance of velocity in east direction [m/s]
cov_w = 0; % Covariance of velocity in down direction [m/s]
cov_lat = 0*(1/6051800); % Covariance of position in latitude direction [m]
cov_lon = 0*(1/6051800); % Covariance of position in longitude direction [m]
cov_h = 0; % Covariance of position in height direction [m]
cov_gyro_bias = 0*(pi/180)*(1/3600); % Covariance of gyroscope bias [rad/s]
cov_accel_bias = 0*(8.858/1000); % Covariance of accelerometer bias [m/s^2]

% Particle Filter
N = 5000; % Number of particles

%% Venus Environment

% Display current progress
disp('Venus Environment...')

% Topography
load('topography.mat','longitudes','latitudes','topography')
topography = griddedInterpolant({longitudes,latitudes},topography');

% Anemology
load('anemology.mat','anemology','longitudes','latitudes','heights')
e_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{1},[2 1 3]));
n_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{2},[2 1 3]));
u_wind = griddedInterpolant({longitudes,latitudes,heights},permute(anemology{3},[2 1 3]));

% Barometry
load('barometry.mat','barometry')
barometry = griddedInterpolant({longitudes,latitudes,heights},permute(barometry,[2 1 3]));

%% Initialization

% Display current progress
disp('Initialization...')

% Establish parallel pool
parpool('threads');

% Time
L = round(time/dt)+1;

% Preallocation
ins_att = zeros(3,L);
ins_vel = zeros(3,L);
ins_pos = zeros(3,L);
ins_pos_xyz = zeros(3,L);
ins_pos_ned = zeros(3,L);
omega_b_ib = zeros(3,L);
bias_omega_b_ib = zeros(3,L);
f_ib_b = zeros(3,L);
bias_f_ib_b = zeros(3,L);
vel_w_enu = zeros(3,L);
E = zeros(15,L);
cov = zeros(15,15,L);
truth = struct;
estimate = struct;
pf = struct;

% Pose
attitude = [0;0;0]; % Initial orientation defined by roll, pitch, and yaw [deg]
velocity = [0;0;0]; % Initial velocity defined in the body frame [m/s]
position = [-90+180*rand(1);-180+360*rand(1);50000+20000*rand(1)]; % Initial position desfined by latitude, longitude, [deg] and height [m]
ins_att(:,1) = deg2rad(attitude); % Convert to rad
ins_vel(:,1) = angle2dcm(ins_att(1,1),ins_att(2,1),ins_att(3,1),'XYZ')*velocity; % Convert to Local-Navigation-Frame
ins_pos(:,1) = [deg2rad(position(1));deg2rad(position(2));position(3)]; % Convert to rad
ins_pos_xyz(:,1) = llh2xyz(ins_pos(:,1)); % Convert from geographic to ECEF cartesian coordinates
ins_pos_ned(:,1) = xyz2ned(ins_pos(:,1),ins_pos_xyz(:,1),ins_pos_xyz(:,1));
x0 = [ins_att(:,1);ins_vel(:,1);ins_pos(:,1);bias_omega_b_ib(:,1);bias_f_ib_b(:,1)];

% Wind
vel_w_enu(:,1) = [e_wind(ins_pos(:,1)');n_wind(ins_pos(:,1)');u_wind(ins_pos(:,1)')];

% Accelerometer
bias_static_accel = bias_static_accel*8.858/1000; % Convert to [m/s^2]
bias_dynamic_accel = bias_dynamic_accel*8.858/1000; % Convert to [m/s^2]
random_noise_accel = random_noise_accel/sqrt(3600)/sqrt(dt); % Convert to [m/s^2]

% Gyroscope
bias_static_gyro = bias_static_gyro*(pi/180)*(1/3600); % Convert to [rad/s]
bias_dynamic_gyro = bias_dynamic_gyro*(pi/180)*(1/3600); % Convert to [rad/s]
random_noise_gyro = random_noise_gyro*(pi/180)*(1/sqrt(3600))/sqrt(dt); % Convert to [rad/s]

% Accelerometer Error
b_as = normrnd(0,bias_static_accel,[3,1]).*ones(3,L);
b_ad = zeros(3,L);
for k = 2:L
    b_ad(:,k) = b_ad(:,k-1)+normrnd(0,bias_dynamic_accel/sqrt(1/dt),[3,1]);
end
w_a = normrnd(0,random_noise_accel,[3,L]);
error_accel = b_as+b_ad+w_a;

% Gyroscope Error
b_gs = normrnd(0,bias_static_gyro,[3,1]).*ones(3,L);
b_gd = zeros(3,L);
for k = 2:L
    b_gd(:,k) = b_gd(:,k-1)+normrnd(0,bias_dynamic_gyro/sqrt(1/dt),[3,1]);
end
w_g = normrnd(0,random_noise_gyro,[3,L]);
error_gyro = b_gs+b_gd+w_g;

% Process Noise
Q = diag([sigma_roll sigma_pitch sigma_yaw sigma_u sigma_v sigma_w sigma_lat sigma_lon sigma_h sigma_gyro_bias sigma_gyro_bias sigma_gyro_bias sigma_accel_bias sigma_accel_bias sigma_accel_bias])*dt^2;

% Measurement Noise
R = diag([sigma_a sigma_b]).^2;

% Initial Distribution
P0 = diag([cov_roll cov_pitch cov_yaw cov_u cov_v cov_w cov_lat cov_lon cov_h cov_gyro_bias cov_gyro_bias cov_gyro_bias cov_accel_bias cov_accel_bias cov_accel_bias]);

% Particle Filter
x = repmat(x0,1,N) + P0 * randn(length(x0),N); % State Matrix
w = ones(N,1)/N; % Particle Weight
q = ones(N,1)/N; % Particle Likelihood
cov(:,:,1) = P0;

%% Simulation

% Iterate for truth [1], estimate [2], and particle filter [3]
for j = 1:3

    % Display Progress
    if j == 1
        disp('Truth Simulation...')
    elseif j == 2
        disp('Noisy Simulation...')
    elseif j == 3
        disp('PF Simulation...')
    end

    % Iterate for all time steps less the first
    for k = 2:L

        % If truth
        if j == 1

            % Anemology
            vel_w_enu(:,k) = [e_wind(ins_pos(:,k-1)');n_wind(ins_pos(:,k-1)');u_wind(ins_pos(:,k-1)')];

            % Flight Model
            [omega_b_ib(:,k),f_ib_b(:,k)] = Flight_Model(ins_att(:,k-1),ins_vel(:,k-1),vel_w_enu(:,k),ins_pos(:,k-1),ins_pos_xyz(:,k-1),dt);

            % Inertial Navigation System
            [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(ins_att(:,k-1),ins_vel(:,k-1),ins_pos(:,k-1),omega_b_ib(:,k),f_ib_b(:,k)*dt,dt);

            % Update States
            ins_att(:,k) = ins_att_plus;
            ins_vel(:,k) = ins_vel_plus;
            ins_pos(:,k) = ins_pos_plus;
            ins_pos_xyz(:,k) = llh2xyz(ins_pos(:,k));
            ins_pos_ned(:,k) = xyz2ned(ins_pos(:,k),ins_pos_xyz(:,k),ins_pos_xyz(:,1));

            % Display Progress
            if(mod(k,0.1*(L-1))==0)
                progress = k/(L-1)*100;
                fprintf('%.0f%% \n',progress);
            end

            % If estimate
        elseif j == 2

            % Add error
            omega_b_ib(:,k) = omega_b_ib(:,k) + error_gyro(:,k);
            f_ib_b(:,k) = f_ib_b(:,k) + error_accel(:,k);

            % Inertial Navigation System
            [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(ins_att(:,k-1),ins_vel(:,k-1),ins_pos(:,k-1),omega_b_ib(:,k),f_ib_b(:,k)*dt,dt);

            % Update States
            ins_att(:,k) = ins_att_plus+normrnd([0 0 0],[Q(1,1) Q(2,2) Q(3,3)])';
            ins_vel(:,k) = ins_vel_plus+normrnd([0 0 0],[Q(4,4) Q(5,5) Q(6,6)])';
            ins_pos(:,k) = ins_pos_plus+normrnd([0 0 0],[Q(7,7) Q(8,8) Q(9,9)])';
            ins_pos_xyz(:,k) = llh2xyz(ins_pos(:,k));
            ins_pos_ned(:,k) = xyz2ned(ins_pos(:,k),ins_pos_xyz(:,k),ins_pos_xyz(:,1));

            % Display Progress
            if(mod(k,0.1*(L-1))==0)
                progress = k/(L-1)*100;
                fprintf('%.0f%% \n',progress);
            end

            % If particle filter
        elseif j == 3

            % Inertial Navigation System of each particle
            orient = x(1:3,:);
            vel = x(4:6,:);
            pos = x(7:9,:);
            gyro_bias = x(10:12,:);
            gyro = omega_b_ib(:,k)-gyro_bias;
            accel_bias = x(13:15,:);
            accel = f_ib_b(:,k)-accel_bias;
            for l = 1:N
                [ins_att_plus,ins_vel_plus,ins_pos_plus] = Inertial_Navigation_System(orient(:,l),vel(:,l),pos(:,l),gyro(:,l),accel(:,l)*dt,dt);
                x(:,l) = [ins_att_plus;ins_vel_plus;ins_pos_plus;gyro_bias(:,l);accel_bias(:,l)];
            end

            x(1,:) = x(1,:)+normrnd(0,Q(1,1),[1,N]);
            x(2,:) = x(2,:)+normrnd(0,Q(2,2),[1,N]);
            x(3,:) = x(3,:)+normrnd(0,Q(3,3),[1,N]);
            x(4,:) = x(4,:)+normrnd(0,Q(4,4),[1,N]);
            x(5,:) = x(5,:)+normrnd(0,Q(5,5),[1,N]);
            x(6,:) = x(6,:)+normrnd(0,Q(6,6),[1,N]);
            x(7,:) = x(7,:)+normrnd(0,Q(7,7),[1,N]);
            x(8,:) = x(8,:)+normrnd(0,Q(8,8),[1,N]);
            x(9,:) = x(9,:)+normrnd(0,Q(9,9),[1,N]);
            x(10,:) = x(10,:)+normrnd(0,Q(10,10),[1,N]);
            x(11,:) = x(11,:)+normrnd(0,Q(11,11),[1,N]);
            x(12,:) = x(12,:)+normrnd(0,Q(12,12),[1,N]);
            x(13,:) = x(13,:)+normrnd(0,Q(13,13),[1,N]);
            x(14,:) = x(14,:)+normrnd(0,Q(14,14),[1,N]);
            x(15,:) = x(15,:)+normrnd(0,Q(15,15),[1,N]);

            % If update is available
            if mod((k-1)*dt,dt_pf) == 0

                % Measurements with noise
                alt = topography(rad2deg(truth.pos(2,k)),rad2deg(truth.pos(1,k))) + normrnd(0,R(1,1));
                pres = barometry(rad2deg(truth.pos(2,k)),rad2deg(truth.pos(1,k)),truth.pos(3,k)) + normrnd(0,R(2,2));
                yNow = repmat([alt pres],N,1);

                % Estimated measurements of each particle
                yhat(:,1) = topography(rad2deg(x(8,:)'),rad2deg(x(7,:)'));
                yhat(:,2) = barometry(rad2deg(x(8,:)'),rad2deg(x(7,:)'),x(9,:)');

                % Likelihood of each particle
                q = normpdf(yNow(:,1),yhat(:,1),r*sigma_a).*normpdf(yNow(:,2),yhat(:,2),r*sigma_b);

                % Multiply previous weight
                q = q .* w;

                % Effective particle count
                if sum(q) == 0
                    break
                end

                % Normalize the weights
                q = q ./ sum(q);

                % Copy the weights
                w = q;

                % If count below threshold...
                if 1/sum(w.^2) < 0.5*N

                    % Resample
                    [x,w] = MSVResampling(x,w);

                end
            end

            % State predicted by Particle Filter
            E(:,k) = [sum(q.*x(1,:)') sum(q.*x(2,:)') sum(q.*x(3,:)') sum(q.*x(4,:)') sum(q.*x(5,:)') sum(q.*x(6,:)') sum(q.*x(7,:)') sum(q.*x(8,:)') sum(q.*x(9,:)') sum(q.*x(10,:)') sum(q.*x(11,:)') sum(q.*x(12,:)') sum(q.*x(13,:)') sum(q.*x(14,:)') sum(q.*x(15,:)')]';
            cov(:,:,k) = diag([sum(q.*((x(1,:)-E(1,k)).*(x(1,:)-E(1,k)))') sum(q.*((x(2,:)-E(2,k)).*(x(2,:)-E(2,k)))') sum(q.*((x(3,:)-E(3,k)).*(x(3,:)-E(3,k)))') sum(q.*((x(4,:)-E(4,k)).*(x(4,:)-E(4,k)))') sum(q.*((x(5,:)-E(5,k)).*(x(5,:)-E(5,k)))') sum(q.*((x(6,:)-E(6,k)).*(x(6,:)-E(6,k)))') sum(q.*((x(7,:)-E(7,k)).*(x(7,:)-E(7,k)))') sum(q.*((x(8,:)-E(8,k)).*(x(8,:)-E(8,k)))') sum(q.*((x(9,:)-E(9,k)).*(x(9,:)-E(9,k)))') sum(q.*((x(10,:)-E(10,k)).*(x(10,:)-E(10,k)))') sum(q.*((x(11,:)-E(11,k)).*(x(11,:)-E(11,k)))') sum(q.*((x(12,:)-E(12,k)).*(x(12,:)-E(12,k)))') sum(q.*((x(13,:)-E(13,k)).*(x(13,:)-E(13,k)))') sum(q.*((x(14,:)-E(14,k)).*(x(14,:)-E(14,k)))') sum(q.*((x(15,:)-E(15,k)).*(x(15,:)-E(15,k)))')]);
            ins_att(:,k) = E(1:3,k);
            ins_vel(:,k) = E(4:6,k);
            ins_pos(:,k) = E(7:9,k);
            bias_omega_b_ib(:,k) = E(10:12,k);
            bias_f_ib_b(:,k) = E(13:15,k);
            ins_pos_xyz(:,k) = llh2xyz(ins_pos(:,k));
            ins_pos_ned(:,k) = xyz2ned(ins_pos(:,k),ins_pos_xyz(:,k),ins_pos_xyz(:,1));

            % Display Progress
            if(mod(k,0.01*(L-1))==0)
                progress = k/(L-1)*100;
                fprintf('%.0f%% \n',progress);
            end
        end
    end

    % If truth
    if j == 1

        % Store data
        truth.att = ins_att;
        truth.vel = ins_vel;
        truth.pos = ins_pos;
        truth.pos_xyz = ins_pos_xyz;
        truth.pos_ned = ins_pos_ned;
        truth.omega_b_ib = omega_b_ib;
        truth.bias_omega_b_ib = error_gyro;
        truth.f_ib_b = f_ib_b;
        truth.bias_f_ib_b = error_accel;
        truth.vel_w_enu = vel_w_enu;
        truth.state = [ins_att;ins_vel;ins_pos;error_gyro;error_accel;];

        % Clear data
        ins_att(:,2:end) = zeros(size(ins_att(:,2:end)));
        ins_vel(:,2:end) = zeros(size(ins_vel(:,2:end)));
        ins_pos(:,2:end) = zeros(size(ins_pos(:,2:end)));
        ins_pos_xyz(:,2:end) = zeros(size(ins_pos_xyz(:,2:end)));
        ins_pos_ned(:,2:end) = zeros(size(ins_pos_ned(:,2:end)));

        % If estimate
    elseif j == 2

        % Store Data
        estimate.att = ins_att;
        estimate.vel = ins_vel;
        estimate.pos = ins_pos;
        estimate.bias_omega_b_ib = zeros(3,L);
        estimate.bias_f_ib_b = zeros(3,L);
        estimate.pos_xyz = ins_pos_xyz;
        estimate.pos_ned = ins_pos_ned;
        estimate.omega_b_ib = omega_b_ib;
        estimate.f_ib_b = f_ib_b;
        estimate.state = [ins_att;ins_vel;ins_pos;zeros(3,L);zeros(3,L)];

        % Clear data
        ins_att(:,2:end) = zeros(size(ins_att(:,2:end)));
        ins_vel(:,2:end) = zeros(size(ins_vel(:,2:end)));
        ins_pos(:,2:end) = zeros(size(ins_pos(:,2:end)));
        ins_pos_xyz(:,2:end) = zeros(size(ins_pos_xyz(:,2:end)));
        ins_pos_ned(:,2:end) = zeros(size(ins_pos_ned(:,2:end)));

        % If particle filter
    elseif j == 3

        % Store Data
        pf.att = ins_att;
        pf.vel = ins_vel;
        pf.pos = ins_pos;
        pf.bias_omega_b_ib = bias_omega_b_ib;
        pf.bias_f_ib_b = bias_f_ib_b;
        pf.pos_xyz = ins_pos_xyz;
        pf.pos_ned = ins_pos_ned;
        pf.state = [ins_att;ins_vel;ins_pos;bias_omega_b_ib;bias_f_ib_b];
        pf.E = E;
        pf.Cov = cov;

    end

    % Save Data
    trial = {truth,estimate,pf};

end

% Stop timer
duration = toc;

% Save data
save('Test_1.mat', 'trial', '-v7.3')
% scp heath_cottrill@157.182.212.133:/home/users/heath_cottrill/Venus_Simulation/Test_1.mat ~/Desktop

% Shutdown Cluster
delete(gcp('nocreate'))

% Display current progress
disp('Simulation Complete!')
