clc; clear; close all;

% Simulation Parameters
numDrones = 2;        % Number of UAVs
numUsers = 5;         % Number of ground users
baseStation = [0, 0, 0]; % Base Station (x, y, z)
areaSize = 1000;      % Simulation area (meters)
maxTime = 50;         % Simulation time (seconds)
timeStep = 1;         % Time step (seconds)
droneSpeed = 20;      % UAV speed (m/s)
droneAltitudes = [100, 200]; % UAV altitudes in meters

% Communication Parameters
transmitPower = 10;   % Transmit power (dBm)
pathLossExponent = 2.5; % Path loss exponent (adjusted for altitude)
noisePower = -90;     % Noise power (dBm)
bandwidth = 1e6;      % Bandwidth in Hz

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
