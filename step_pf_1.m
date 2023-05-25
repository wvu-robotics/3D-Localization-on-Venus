%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% step_pf_1.m
% 
% Purpose: Update Terrain-aided Navigation Particle Filter
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

%% Save Previous Results

pf.pre_xf_1 = pf.xf_1;
pf.pre_x_1 = pf.x_1;

%% Obtain Measurement

pf.meas_1 = f(agent.tpx(simulation.i), agent.tpy(simulation.i))*ones(pf.ny_1,1);
pf.altitude(simulation.i) = pf.meas_1;
pf.meas_1 = pf.meas_1 + simulation.sigma_Range * randn(size(pf.meas_1)); % add ranging measurement noise
pf.yNow_1 = pf.meas_1;  %ranging measurement from rangefinder

%% Prediction using Odom Measurement

pf.v_1 = repmat(agent.vel(simulation.i),1,pf.npf_1)+simulation.sigma_Velocity_temp*randn(1,pf.npf_1);    %noised velocity measurement
pf.raw_v_1 = agent.vel(simulation.i);
pf.droll_1 = repmat(agent.droll(simulation.i),1,pf.npf_1)+simulation.sigma_Roll_Rate_temp*randn(1,pf.npf_1); %noised yaw rate measurement
pf.raw_droll_1 = agent.droll(simulation.i);
pf.dpitch_1 = repmat(agent.dpitch(simulation.i),1,pf.npf_1)+simulation.sigma_Pitch_Rate_temp*randn(1,pf.npf_1); %noised yaw rate measurement
pf.raw_dpitch_1 = agent.dpitch(simulation.i);
pf.dyaw_1 = repmat(agent.dyaw(simulation.i),1,pf.npf_1)+simulation.sigma_Yaw_Rate_temp*randn(1,pf.npf_1); %noised yaw rate measurement
pf.raw_dyaw_1 = agent.dyaw(simulation.i);

pf.x_1 = m_formulas.fnf(simulation.Ts,pf.dpitch_1,pf.droll_1,pf.dyaw_1,pf.x_1(5,:),pf.x_1(4,:),pf.x_1(1,:),pf.x_1(2,:),pf.x_1(6,:),pf.x_1(3,:),pf.v_1);

%% Particle Filter

if(mod((simulation.i)*simulation.Ts,simulation.update_Ts_1==0))  % pf works when performing ranging measurement
    
    pf.yhat_1 = []; % estimated observation
    pf.yhat_1 = [f(pf.x_1(1,:)', pf.x_1(2,:)')];
    pf.yhat_1 = pf.yhat_1';
    
    % calculate residue in pf
    pf.e_1 = repmat(pf.yNow_1,1,pf.npf_1) - pf.yhat_1;
    
    % calculate the likelihood by multiplying all likelihoods
    pf.q_1 = ones(pf.npf_1,1);
    pf.q_1 = pf.q_1 .* normpdf(pf.e_1(1,:),0,simulation.sigma_Range*10)'; % Why multiplied by 10?
    
    % check divergence
    if(sum(pf.q_1) < pf.divergence_threshold_1)
        pf.q_1 = zeros(pf.npf_1,1);
    end
    
    % calculate new weights
    pf.q_1 = pf.q_1 .* pf.w_1; % multiply previous weight
    pf.q_1 = pf.q_1 ./ sum(pf.q_1); % normalize the important weights
    pf.w_1 = pf.q_1; % copy particles' weights
    
    for i = 1:2
        pf.xf_1(i,1) = sum(pf.q_1.*pf.x_1(i,:)'); %average weighted
    end

    pf.xf_1(3,1) = agent.pz(simulation.i);
    pf.xf_1(4,1) = agent.roll(simulation.i);
    pf.xf_1(5,1) = agent.pitch(simulation.i);
    pf.xf_1(6,1) = agent.yaw(simulation.i);

    % if the number of effective particles below threshold, re sample
    pf.N_eff_1 = 1/sum(pf.q_1.^2);  %calculate effective particles
    
    if pf.N_eff_1 < simulation.threshold_resample
        pf.index_1 = resampleMSV(pf.q_1);
        pf.x_1 = pf.x_1(:,pf.index_1);
        pf.w_1 = ones(pf.npf_1,1);
    end
    
else % without particle filter update, using dead reckoning update
    [x,y,~,~,~,~] = fn_propagate_global_pose(pf.xf_1(1),...
        pf.xf_1(2),...
        pf.xf_1(3),...
        pf.xf_1(4),...
        pf.xf_1(5),...
        pf.xf_1(6),...
        agent.vel(simulation.i),...
        agent.droll(simulation.i),...
        agent.dpitch(simulation.i),...
        agent.dyaw(simulation.i),...
        simulation.Ts);
    pf.xf_1(1) = x;
    pf.xf_1(2) = y;
    pf.xf_1(3) = agent.pz(simulation.i);
    pf.xf_1(4) = agent.roll(simulation.i);
    pf.xf_1(5) = agent.pitch(simulation.i);
    pf.xf_1(6) = agent.yaw(simulation.i);
end

%% Clean

clear x y z roll pitch yaw