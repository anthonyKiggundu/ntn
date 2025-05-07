function UAV_Script_BiggerGap()
% UAV_SCRIPT_BIGGERGAP
%
% Demonstrates a scenario configured to produce a
% much bigger capacity/EE gap for the UAV-based approach.
%
% Key changes:
% 1) GBS & LEO are weaker or more limited (wG_max=8, wL_max=3),
%    plus heavy shadowing => many users have near-zero LEO SNR.
% 2) Fraction ~ 70% poor LEO coverage.
% 3) UAV has bigger p_max=35 dBm, extra directional gain -15 dB offset,
%    smaller p_hover=10 => better energy usage.
% 4) On-demand UAV: only powers up if leftover>0.
%
% We compare 4 schemes:
%   (A) GBS-Only
%   (B) LEO-GBS
%   (C) EGC-SAGIN (2-phase AF, leftover with UAV)
%   (D) CUD(MRC)
%
% The sum capacity uses linear scale, the energy efficiency uses log scale for better clarity.

clc; clear; close all; rng(0,'twister');

numUsersVec = 0:1:10;  % range of user counts
nTrials     = 30;         % Monte Carlo

C_gbs  = zeros(1,length(numUsersVec));
C_leo  = zeros(1,length(numUsersVec));
C_egc  = zeros(1,length(numUsersVec));
C_mrc  = zeros(1,length(numUsersVec));

EE_gbs = zeros(1,length(numUsersVec));
EE_leo = zeros(1,length(numUsersVec));
EE_egc = zeros(1,length(numUsersVec));
EE_mrc = zeros(1,length(numUsersVec));

idx=0;
for N = numUsersVec
    idx=idx+1;
    
    sumCg=0; sumCl=0; sumCe=0; sumCm=0;
    sumEg=0; sumEl=0; sumEe=0; sumEm=0;
    
    for t=1:nTrials
        [cg,cl,ce,cm, eg,el,ee,em] = leftoverOnePass_biggerGap(N);
        sumCg= sumCg+ cg; sumCl= sumCl+ cl;
        sumCe= sumCe+ ce; sumCm= sumCm+ cm;
        sumEg= sumEg+ eg; sumEl= sumEl+ el;
        sumEe= sumEe+ ee; sumEm= sumEm+ em;
    end
    
    C_gbs(idx)= sumCg/(nTrials*1e9);
    C_leo(idx)= sumCl/(nTrials*1e9);
    C_egc(idx)= sumCe/(nTrials*1e9);
    C_mrc(idx)= sumCm/(nTrials*1e9);
    
    EE_gbs(idx)= sumEg/nTrials;
    EE_leo(idx)= sumEl/nTrials;
    EE_egc(idx)= sumEe/nTrials;
    EE_mrc(idx)= sumEm/nTrials;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Call these somewhere to adjust the drone altitude %%%% 
% currentAltitude = 100; % initial altitude of the UAV
% deltaAltitude = 50;    % change in altitude
% newAltitude = changeUAVAltitude(currentAltitude, deltaAltitude);
% disp(['New UAV Altitude: ', num2str(newAltitude), ' meters']);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot Sum Capacity (linear)
figure('Name','Sum Capacity - UAV Amplified Gains');
plot(numUsersVec, C_gbs,'b-o','LineWidth',1.5); hold on;
plot(numUsersVec, C_leo,'r-s','LineWidth',1.5);
plot(numUsersVec, C_egc,'g-d','LineWidth',1.5);
plot(numUsersVec, C_mrc,'m-*','LineWidth',1.5);
grid on;
xlabel('Number of Users'); ylabel('Sum Capacity (Gbps)');
legend('GBS-Only','LEO-GBS','EGC-SAGIN','CUD(MRC)','Location','best');
title('GBS vs. LEO vs. UAV(ECG/MRC) - Summed Capacity');
% set(gca,'YScale','log');  % log scale on y-axis
% Plot Energy Efficiency (log y-axis)
figure('Name','Energy Efficiency (Log) - UAV Amplified Gains');
plot(numUsersVec, EE_gbs,'b-o','LineWidth',1.5); hold on;
plot(numUsersVec, EE_leo,'r-s','LineWidth',1.5);
plot(numUsersVec, EE_egc,'g-d','LineWidth',1.5);
plot(numUsersVec, EE_mrc,'m-*','LineWidth',1.5);
grid on;
xlabel('Number of Users'); ylabel('Energy Efficiency (bps/W)');
legend('GBS-Only','LEO-GBS','EGC-SAGIN','CUD(MRC)','Location','best');
title('GBS vs. LEO vs. UAV(ECG/MRC) - Energy Efficiency');
% set(gca,'YScale','log');  % log scale on y-axis

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Cg,Cl,Ce,Cm, Eg,El,Ee,Em] = leftoverOnePass_biggerGap(N)
% leftoverOnePass_biggerGap
%
% Scenario forcibly set to highlight UAV approach:
% - wG_max=8, wL_max=3 => quickly saturate GBS/LEO
% - ~70% users have near zero LEO coverage => leftover
% - UAV => p_max=35 dBm, snr_th=7 dB, directional offset=15 dB
% - p_hover=10 => minimal overhead
% - On-demand UAV => only used if leftover>0

Area_side= 1000;
wG_max=30;
wL_max=30;
wU_max=30;

SNR_th_dB=7;  % raise threshold => UAV tries for ~5 times bigger SNR than 3 dB
snr_th_lin= 10^((SNR_th_dB)/10);

p_max_dBm=35;  % 35 dBm => ~3.16 W
p_max_lin= 10^((p_max_dBm-30)/10);

B=20e6;
N0dBm= -174 + 10*log10(B);
N0_lin= 10^((N0dBm-30)/10);

% GBS & LEO typical powers
Ptx_GBS=45;  % 35 dBm => ~3.16 W (weak for a GBS)
Ptx_LEO=45;  % 45 dBm => 31.6 W
p_lin_GBS= 10^((Ptx_GBS-30)/10);
p_lin_LEO= 10^((Ptx_LEO-30)/10);

p_circuit=10;
p_hover=10;  % smaller overhead => helps UAV's EE
zeta=1e-4;   % smaller => bigger combine difference

% Place users
x= rand(1,N)*Area_side;
y= rand(1,N)*Area_side;

% GBS->User => small range [0..2] => quite NLoS
snr_gbs_lin= 2*rand(1,N);

% LEO->User => 70% no coverage => [0..0.2], 30% moderate => [1..3]
fracNoCov= 0.1; 
nNoCov= round(N*fracNoCov);
idxAll= randperm(N);
idxNoCov= idxAll(1:nNoCov);
idxLEOMod= idxAll(nNoCov+1:end);

snr_leo_lin= zeros(1,N);
snr_leo_lin(idxNoCov)= 0.0 + 0.2*rand(1,nNoCov); 
snr_leo_lin(idxLEOMod)= 1 + 2*rand(1, N-nNoCov);

% LEO->UAV => moderate => [1..6]
snr_js_lin= 1 + 5*rand(1,N);

% UAV->User => strong LoS => pathloss ~ [70..80], minus 15 => [55..65]
PL_uav_dB= (70 + 10*randn(1,N)) -15;

% Step (1) GBS association
delta_g= zeros(1,N); usedG=0;
for i=1:N
   if 10*log10(snr_gbs_lin(i))>= SNR_th_dB && usedG< wG_max
       delta_g(i)=1; usedG=usedG+1;
   end
end
OmegaEx= find(delta_g==0);

% Step (2) leftover tries LEO
delta_l= zeros(1,N); usedL=0;
for i=OmegaEx
   if 10*log10(snr_leo_lin(i))>= SNR_th_dB && usedL< wL_max
       delta_l(i)=1; usedL=usedL+1;
   end
end
OmegaEx2= setdiff(OmegaEx, find(delta_l==1));

% Step (3) leftover tries UAV => on-demand
delta_u= zeros(1,N); usedU=0;
p_alloc= zeros(1,N);
uavOn = false;

if ~isempty(OmegaEx2)
    uavOn=true;
    for i=OmegaEx2
       if usedU>= wU_max, break; end
       if snr_js_lin(i)< 0.5, continue; end
       [p_req, ~] = computeMinPowerLoS(PL_uav_dB(i), snr_th_lin, p_max_lin, N0_lin);
       if p_req>0
          delta_u(i)=1; usedU=usedU+1; p_alloc(i)= p_req;
       end
    end
end

% (A) GBS-Only => sum GBS
Csum_GBS=0;
for i=1:N
   if delta_g(i)==1
       Csum_GBS= Csum_GBS + B*log2(1+ snr_gbs_lin(i));
   end
end
p_gbsOnly= p_lin_GBS + p_circuit;
Eg= Csum_GBS / p_gbsOnly;

% (B) LEO-GBS => GBS or LEO
Csum_LEO=0;
for i=1:N
   if delta_g(i)==1
       Csum_LEO= Csum_LEO + B*log2(1+ snr_gbs_lin(i));
   else
       Csum_LEO= Csum_LEO + B*log2(1+ snr_leo_lin(i));
   end
end
p_leoOnly= p_lin_GBS + p_lin_LEO + p_circuit;
El= Csum_LEO / p_leoOnly;

% (C) EGC-SAGIN => leftover uses UAV => 2-phase
Csum_EGC=0;
for i=1:N
   if delta_u(i)==1
       snr_ij= computeSNRwithPowerLoS(PL_uav_dB(i), p_alloc(i), N0_lin);
       gamma_egc= ( snr_leo_lin(i) + snr_js_lin(i)* snr_ij ) / ( snr_ij+ zeta );
       Csum_EGC= Csum_EGC + (B/2)* log2(1+ gamma_egc);
   elseif delta_g(i)==1
       Csum_EGC= Csum_EGC + B*log2(1+ snr_gbs_lin(i));
   else
       Csum_EGC= Csum_EGC + B*log2(1+ snr_leo_lin(i));
   end
end

% (D) CUD(MRC)
Csum_MRC=0;
for i=1:N
   if delta_u(i)==1
       snr_ij= computeSNRwithPowerLoS(PL_uav_dB(i), p_alloc(i), N0_lin);
       gamma_mrc= snr_leo_lin(i) + ( snr_js_lin(i)* snr_ij )/( snr_ij + zeta );
       Csum_MRC= Csum_MRC + (B/2)* log2(1+ gamma_mrc);
   elseif delta_g(i)==1
       Csum_MRC= Csum_MRC + B*log2(1+ snr_gbs_lin(i));
   else
       Csum_MRC= Csum_MRC + B*log2(1+ snr_leo_lin(i));
   end
end

% total power for EGC/MRC => GBS + LEO + UAV if on
sumPUAV= sum(p_alloc);
if uavOn
   p_egc= p_lin_GBS + p_lin_LEO + sumPUAV + p_hover + p_circuit;
else
   p_egc= p_lin_GBS + p_lin_LEO + p_circuit;
end
Ee= Csum_EGC / p_egc;
Em= Csum_MRC / p_egc;

Cg= Csum_GBS; Cl= Csum_LEO; Ce= Csum_EGC; Cm= Csum_MRC;
Eg= Eg; El= El; Ee= Ee; Em= Em;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [p_req, finalSNR] = computeMinPowerLoS(PLdB, snr_lin_th, p_max_lin, N0_lin)
p_min= snr_lin_th*N0_lin* 10^((PLdB)/10);
if p_min> p_max_lin
   p_req=0; finalSNR=0; return;
end
p_req= p_min;
finalSNR= computeSNRwithPowerLoS(PLdB, p_req, N0_lin);
if finalSNR< 0.8* snr_lin_th
   p_req=0; finalSNR=0;
end
end

function snr_lin= computeSNRwithPowerLoS(PLdB, p_lin, N0_lin)
rx_dB= 10*log10(p_lin*1e3) - PLdB;
rx_lin= 10^((rx_dB-30)/10);
snr_lin= rx_lin/ N0_lin;
if snr_lin<0, snr_lin=0; end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [newAltitude, pathLoss] = changeUAVAltitude(currentAltitude, deltaAltitude, pathLossExponent, baseDistance)
% CHANGEUAVALTITUDE
% Change the altitude of the UAV and calculate the path loss.
%
% Inputs:
%   currentAltitude   - The current altitude of the UAV (in meters).
%   deltaAltitude     - The change in altitude (in meters).
%   pathLossExponent  - The path-loss exponent (dimensionless, e.g., 2-6).
%   baseDistance      - The horizontal distance from the UAV to the receiver (in meters).
%
% Outputs:
%   newAltitude - The new altitude of the UAV (in meters).
%   pathLoss    - The calculated path loss (in dB).

% Update the UAV altitude
newAltitude = currentAltitude + deltaAltitude;

% Ensure the new altitude is within bounds (e.g., 0 to 500 meters)
minAltitude = 0;
maxAltitude = 500;
if newAltitude < minAltitude
    newAltitude = minAltitude;
elseif newAltitude > maxAltitude
    newAltitude = maxAltitude;
end

% Calculate the 3D distance
distance3D = sqrt(baseDistance^2 + newAltitude^2);

% Calculate the path loss in dB
pathLoss = 10 * pathLossExponent * log10(distance3D + 1);

% Display the new altitude and path loss
disp(['New UAV Altitude: ', num2str(newAltitude), ' meters']);
disp(['Path Loss: ', num2str(pathLoss), ' dB']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Simulating live %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize UAV and User Positions (3D)
drones = [areaSize * (rand(numDrones, 2) - 0.5), droneAltitudes']; % Drone positions in 3D
users = [areaSize * (rand(numUsers, 2) - 0.5), zeros(numUsers, 1)]; % Users at ground level

% Plot Initial Network Setup
figure; hold on; grid on;
plot3(baseStation(1), baseStation(2), baseStation(3), 'ks', 'MarkerSize', 10, 'LineWidth', 2); % Base Station
dronePlots = plot3(drones(:,1), drones(:,2), drones(:,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Drones
userPlots = plot3(users(:,1), users(:,2), users(:,3), 'bx', 'MarkerSize', 8, 'LineWidth', 2); % Users
legend('Base Station', 'Drones', 'Users');
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Altitude (m)');
title('UAV Network Simulation with Altitude-Based QoS');
xlim([-areaSize/2, areaSize/2]); ylim([-areaSize/2, areaSize/2]); zlim([0, 500]);

% Simulation Loop
for t = 1:timeStep:maxTime
    % Update drone positions (move towards users)
    for d = 1:numDrones
        targetUser = users(mod(d-1, numUsers) + 1, :); % Assign each drone to a user
        direction = (targetUser(1:2) - drones(d, 1:2)) / norm(targetUser(1:2) - drones(d, 1:2) + 1e-6); % 2D direction
        drones(d, 1:2) = drones(d, 1:2) + droneSpeed * direction * timeStep; % Move in x-y plane
    end
    
    % Compute QoS Metrics (SNR & Data Rate) considering altitude
    for u = 1:numUsers
        for d = 1:numDrones
            dist = norm(users(u, :) - drones(d, :)); % 3D Distance from user to drone
            pathLoss = 10 * pathLossExponent * log10(dist + 1);
            receivedPower = transmitPower - pathLoss; % Received power (dBm)
            SNR = receivedPower - noisePower; % Signal-to-Noise Ratio
            dataRate = bandwidth * log2(1 + 10^(SNR/10)); % Shannon Capacity
            
            % Display QoS metrics
            fprintf('User %d -> Drone %d (Altitude: %.0f m): Distance=%.2f m, SNR=%.2f dB, Data Rate=%.2f Mbps\n', ...
                u, d, drones(d, 3), dist, SNR, dataRate / 1e6);
        end
    end
    
    % Compute Drone-to-Base-Station QoS
    for d = 1:numDrones
        distBS = norm(drones(d, :) - baseStation);
        pathLossBS = 10 * pathLossExponent * log10(distBS + 1);
        receivedPowerBS = transmitPower - pathLossBS;
        SNR_BS = receivedPowerBS - noisePower;
        dataRateBS = bandwidth * log2(1 + 10^(SNR_BS/10));
        
        fprintf('Drone %d (Altitude: %.0f m) -> Base Station: Distance=%.2f m, SNR=%.2f dB, Data Rate=%.2f Mbps\n', ...
            d, drones(d, 3), distBS, SNR_BS, dataRateBS / 1e6);
    end
    
    % Update visualization
    set(dronePlots, 'XData', drones(:,1), 'YData', drones(:,2), 'ZData', drones(:,3));
    title(['Time Step: ', num2str(t), ' sec']);
    pause(0.2); % Pause for visualization
end

disp('Simulation complete.');
