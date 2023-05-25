%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% step_pf_2.m
% 
% Purpose: Update pose based on propagation function
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

pf.pre_xf_2 = pf.xf_2;
pf.pre_x_2 = pf.x_2;

%% Obtain Measurement

pf.meas_2 = air_pressure(agent.tpz(simulation.i)*ones(pf.ny_2,1));
measured_pressure = pf.meas_2 + simulation.sigma_Pressure * randn(size(pf.meas_2));
if measured_pressure < 0.0369
    pf.meas_2 = 0.0369;
else
    pf.meas_2 = measured_pressure; % add ranging measurement noise
end
pf.yNow_2 = pf.meas_2;  %ranging measurement from rangefinder

%% Prediction using Odom Measurement

pf.v_2 = repmat(agent.vel(simulation.i),1,pf.npf_2)+simulation.sigma_Velocity_temp*randn(1,pf.npf_2);    %noised velocity measurement
pf.raw_v_2 = agent.vel(simulation.i);
pf.droll_2 = repmat(agent.droll(simulation.i),1,pf.npf_2)+simulation.sigma_Roll_Rate_temp*randn(1,pf.npf_2); %noised yaw rate measurement
pf.raw_droll_2 = agent.droll(simulation.i);
pf.dpitch_2 = repmat(agent.dpitch(simulation.i),1,pf.npf_2)+simulation.sigma_Pitch_Rate_temp*randn(1,pf.npf_2); %noised yaw rate measurement
pf.raw_dpitch_2 = agent.dpitch(simulation.i);
pf.dyaw_2 = repmat(agent.dyaw(simulation.i),1,pf.npf_2)+simulation.sigma_Yaw_Rate_temp*randn(1,pf.npf_2); %noised yaw rate measurement
pf.raw_dyaw_2 = agent.dyaw(simulation.i);

pf.x_2 = m_formulas.fnf(simulation.Ts,pf.dpitch_2,pf.droll_2,pf.dyaw_2,pf.x_2(5,:),pf.x_2(4,:),pf.x_2(1,:),pf.x_2(2,:),pf.x_2(6,:),pf.x_2(3,:),pf.v_2);

%% Particle Filter

if(mod((simulation.i)*simulation.Ts,simulation.update_Ts_2==0))  % pf works when performing pressure measurement
    
    pf.yhat_2 = []; % estimated observation
    pf.yhat_2 = [air_pressure(pf.x_2(3,:)')];
    pf.yhat_2 = pf.yhat_2';
    
    % calculate residue in pf
    pf.e_2 = repmat(pf.yNow_2,1,pf.npf_2) - pf.yhat_2;
    
    % calculate the likelihood by multiplying all likelihoods
    pf.q_2 = ones(pf.npf_2,1);
    pf.q_2 = pf.q_2 .* normpdf(pf.e_2(1,:),0,simulation.sigma_Pressure*10)'; % Why multiplied by 10?
    
    % check divergence
    if(sum(pf.q_2) < pf.divergence_threshold_2)
        pf.q_2 = zeros(pf.npf_2,1);
    end
    
    % calculate new weights
    pf.q_2 = pf.q_2 .* pf.w_2; % multiply previous weight
    pf.q_2 = pf.q_2 ./ sum(pf.q_2); % normalize the important weights
    pf.w_2 = pf.q_2; % copy particles' weights
    
    for i = 3
        pf.xf_2(i,1) = sum(pf.q_2.*pf.x_2(i,:)'); %average weighted
    end

    if isnan(pf.xf_2(3))
        disp('hi')
    end

    pf.xf_2(1,1) = agent.px(simulation.i);
    pf.xf_2(2,1) = agent.py(simulation.i);
    pf.xf_2(4,1) = agent.roll(simulation.i);
    pf.xf_2(5,1) = agent.pitch(simulation.i);
    pf.xf_2(6,1) = agent.yaw(simulation.i);
    
    % if the number of effective particles below threshold, re sample
    pf.N_eff_2 = 1/sum(pf.q_2.^2);  %calculate effective particles
    
    if pf.N_eff_2 < simulation.threshold_resample
        pf.index_2 = resampleMSV(pf.q_2);
        pf.x_2 = pf.x_2(:,pf.index_2);
        pf.w_2 = ones(pf.npf_2,1);
    end
    
else % without particle filter update, using dead reckoning update
    [~,~,z,~,~,~] = fn_propagate_global_pose(pf.xf_2(1),...
        pf.xf_2(2),...
        pf.xf_2(3),...
        pf.xf_2(4),...
        pf.xf_2(5),...
        pf.xf_2(6),...
        agent.vel(simulation.i),...
        agent.droll(simulation.i),...
        agent.dpitch(simulation.i),...
        agent.dyaw(simulation.i),...
        simulation.Ts);
    pf.xf_1(1) = agent.px(simulation.i);
    pf.xf_1(2) = agent.pz(simulation.i);
    pf.xf_2(3) = z;
    pf.xf_1(4) = agent.roll(simulation.i);
    pf.xf_1(5) = agent.pitch(simulation.i);
    pf.xf_1(6) = agent.yaw(simulation.i);
end

%% Clean

clear x y z roll pitch yaw