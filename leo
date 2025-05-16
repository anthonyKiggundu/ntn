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

#################################### Beginning of Python Stuff ################################

import numpy as np
import matplotlib.pyplot as plt

# Simulation class (simplified for this plot)
class EnergyOffloadingSimulation:
    def __init__(self, num_users=50):
        self.N = num_users
        self.L = 10
        self.M = 60
        self.T = 0.02
        self.eta_i = 100
        self.epsilon_i = 5e-6
        self.N0 = 1e-9
        self.G = 1e-3
        self.d0 = 200
        self.NRB_i = 50
        self.B_i = self.NRB_i * 180e3

        np.random.seed(42)
        self.d_i = np.random.uniform(100, 800, self.N)
        self.C_serv_i = self.eta_i * self.M  # eta_s=1, f_i_M=M

    def compute_optimal_alpha(self, nu, beta, D_i):
        Eu_i = self.L * self.epsilon_i * self.eta_i * self.M
        K_i = (np.log(2) * self.N0 * D_i / self.G) * (self.d_i / self.d0)**beta
        r_i = D_i / (self.B_i * self.T)
        alpha_unclipped = (1 / r_i) * np.log2(1 / (K_i * (Eu_i - nu * self.C_serv_i)))
        return np.clip(alpha_unclipped, 0, 1)

    def average_alpha_vs_beta(self, betas, nu):
        avg_alphas = []
        np.random.seed(42)
        D_i = np.random.uniform(1e4, 1e6, self.N)  # fixed D_i for consistency
        for beta in betas:
            alpha = self.compute_optimal_alpha(nu, beta, D_i)
            avg_alphas.append(np.mean(alpha))
        return avg_alphas

# Prepare data
sim = EnergyOffloadingSimulation()
betas = np.linspace(2, 4, 50)
nu = 1e-5
avg_alphas = sim.average_alpha_vs_beta(betas, nu)

# Plot
plt.figure()
plt.plot(betas, avg_alphas, marker='o')
plt.xlabel('Pathloss Exponent (β)')
plt.ylabel('Average Offload Share (α)')
plt.title('Offloading vs Pathloss Exponent')
plt.grid(True)
plt.show()


##### summed_energy versus Pathloss ##########
import numpy as np
import matplotlib.pyplot as plt

# Reuse the simulation class definition
class EnergyOffloadingSimulation:
    def __init__(self, num_users=50):
        self.N = num_users
        self.L = 10
        self.M = 60
        self.S = 8
        self.T = 0.02
        self.eta_i = 100
        self.eta_s = 1
        self.epsilon_i = 5e-6
        self.N0 = 1e-9
        self.G = 1e-3
        self.d0 = 200
        self.NRB_i = 50
        self.B_i = self.NRB_i * 180e3

        np.random.seed(42)
        self.d_i = np.random.uniform(100, 800, self.N)
        self.f_i_M = self.M
        self.C_serv_i = self.eta_s * self.f_i_M

    def compute_Eu_i(self, alpha):
        return (1 - alpha) * self.L * self.epsilon_i * self.eta_i * self.f_i_M

    def compute_Etr_i(self, alpha, beta, D_i):
        factor = (2**(alpha * D_i / (self.B_i * self.T)) - 1)
        P_tr = factor * self.N0 * self.B_i / self.G * (self.d_i / self.d0)**beta
        return P_tr * self.T

    def compute_optimal_alpha(self, nu, beta, D_i):
        Eu_i = self.L * self.epsilon_i * self.eta_i * self.f_i_M
        K_i = (np.log(2) * self.N0 * D_i / self.G) * (self.d_i / self.d0)**beta
        r_i = D_i / (self.B_i * self.T)
        alpha_unclipped = (1 / r_i) * np.log2(1 / (K_i * (Eu_i - nu * self.C_serv_i)))
        return np.clip(alpha_unclipped, 0, 1)

    def total_energy_for_beta(self, beta, nu):
        D_i = np.random.uniform(1e4, 1e6, self.N)
        alpha = self.compute_optimal_alpha(nu, beta, D_i)
        Eu = self.compute_Eu_i(alpha)
        Etr = self.compute_Etr_i(alpha, beta, D_i)
        return np.sum(Eu + Etr)

    def baseline_energy(self, beta, mode):
        # mode 'none' for alpha=0, 'full' for alpha=1
        D_i = np.random.uniform(1e4, 1e6, self.N)
        alpha = np.zeros(self.N) if mode == 'none' else np.ones(self.N)
        Eu = self.compute_Eu_i(alpha)
        Etr = self.compute_Etr_i(alpha, beta, D_i)
        return np.sum(Eu + Etr)

# Set up
sim = EnergyOffloadingSimulation()
betas = np.linspace(2, 4, 50)
nus = [1e-6, 1e-5, 1e-4]

# Compute energies
baseline_none = [sim.baseline_energy(b, 'none') for b in betas]
baseline_full = [sim.baseline_energy(b, 'full') for b in betas]
curves = {nu: [sim.total_energy_for_beta(b, nu) for b in betas] for nu in nus}

# Plot
plt.figure()
plt.plot(betas, baseline_none, linestyle='--', label='No Offload')
plt.plot(betas, baseline_full, linestyle='--', label='Full Offload')
for nu, energies in curves.items():
    plt.plot(betas, energies, label=f'Nu={nu:.0e}')
plt.xlabel('Pathloss Exponent (β)')
plt.ylabel('Total Energy Consumption (mJ)')
plt.title('Energy vs Pathloss: Baselines and Offload Policies')
plt.legend()
plt.grid(True)
plt.show()


############ base code #################
import numpy as np
import pandas as pd
import ace_tools as tools

class EnergyOffloadingSimulation:
    def __init__(self, num_users=50):
        self.N = num_users
        self.L = 10  # data blocks
        self.M = 60  # data elements per block
        self.S = 8   # bits per element
        self.T = 0.02  # time period in seconds
        self.Tpr = 0.001  # server processing time
        self.eta_i = 100  # device computation cycles per unit
        self.eta_s = 1    # server efficiency
        self.epsilon_i = 5e-6  # energy per cycle in mJ
        self.N0 = 1e-9    # noise power spectral density
        self.G = 1e-3     # path loss constant
        self.d0 = 200     # reference distance in meters
        self.NRB_i = 50
        self.B_i = self.NRB_i * 180e3  # bandwidth in Hz

        # Random per-user parameters
        np.random.seed(42)
        self.d_i = np.random.uniform(100, 800, self.N)   # distance from cell center
        self.beta = np.random.uniform(2.0, 4.0, self.N)  # pathloss exponent
        self.D_i = np.random.uniform(1e4, 1e6, self.N)   # data volume in bits

        self.f_i_M = self.M  # linear complexity
        self.C_serv_i = self.eta_s * self.f_i_M  # constant for all users

    def compute_Eu_i(self, alpha):
        # Local energy consumption with offloading share alpha
        return (1 - alpha) * self.L * self.epsilon_i * self.eta_i * self.f_i_M

    def compute_Etr_i(self, alpha):
        # Transmission energy consumption with offloading share alpha
        # Eq: E_tr = P_tr * T, where P_tr = ((2^(alpha*D/(B*T)) - 1)*N0*B/G*(d/d0)^beta)
        factor = (2**(alpha * self.D_i / (self.B_i * self.T)) - 1)
        P_tr = factor * self.N0 * self.B_i / self.G * (self.d_i / self.d0)**self.beta
        return P_tr * self.T

    def compute_optimal_alpha(self, nu):
        Eu_i = self.L * self.epsilon_i * self.eta_i * self.f_i_M
        K_i = (np.log(2) * self.N0 * self.D_i / self.G) * (self.d_i / self.d0)**self.beta
        r_i = self.D_i / (self.B_i * self.T)
        # unclipped alpha
        alpha_unclipped = (1 / r_i) * np.log2(1 / (K_i * (Eu_i - nu * self.C_serv_i)))
        # enforce 0 <= alpha <= 1
        return np.clip(alpha_unclipped, 0, 1)

    def simulate_over_nu(self, nu_values):
        results = []
        for nu in nu_values:
            alphas = self.compute_optimal_alpha(nu)
            Eu = self.compute_Eu_i(alphas)
            Etr = self.compute_Etr_i(alphas)
            total_energy = np.sum(Eu + Etr)
            avg_alpha = np.mean(alphas)
            results.append({
                'nu': nu,
                'avg_alpha': avg_alpha,
                'total_energy_mJ': total_energy
            })
        return pd.DataFrame(results)

# Run simulation
sim = EnergyOffloadingSimulation()
nu_values = np.logspace(-7, -3, num=10)
df_results = sim.simulate_over_nu(nu_values)

# Display results
tools.display_dataframe_to_user("Offloading Simulation Results", df_results)

####  Starts here ###############

import numpy as np

class EnergyOffloadingSimulation:
    def __init__(self, num_users=50):
        self.N = num_users
        self.L = 10  # data blocks
        self.M = 60  # data elements per block
        self.S = 8  # bits per element
        self.T = 0.02  # time period in seconds
        self.Tpr = 0.001  # server processing time
        self.eta_i = 100  # device computation cycles per unit
        self.eta_s = 1  # server efficiency
        self.epsilon_i = 5e-6  # energy per cycle in mJ
        self.N0 = 1e-9  # noise power spectral density
        self.G = 1e-3  # path loss constant
        self.d0 = 200  # reference distance in meters
        self.NRB_i = 50
        self.B_i = self.NRB_i * 180e3  # bandwidth in Hz

        # Random per-user parameters
        self.d_i = np.random.uniform(100, 800, self.N)  # distance from cell center
        self.beta = np.random.uniform(2.0, 4.0, self.N)  # pathloss exponent
        self.D_i = np.random.uniform(1e4, 1e6, self.N)  # data volume in bits

        self.f_i_M = self.M  # linear complexity
        self.C_serv_i = self.eta_s * self.f_i_M  # constant for all users

    def compute_Eu_i(self):
        return self.epsilon_i * self.L * self.eta_i * self.f_i_M * np.ones(self.N)

    def compute_K_i(self):
        return (
            np.log(2)
            * self.N0
            * self.D_i
            / self.G
            * (self.d_i / self.d0) ** self.beta
        )

    def compute_r_i(self):
        return self.D_i / (self.B_i * self.T)

    def compute_optimal_alpha(self, nu):
        Eu_i = self.compute_Eu_i()
        K_i = self.compute_K_i()
        r_i = self.compute_r_i()
        C_serv_i = self.C_serv_i

        alpha_opt = np.maximum((1 / r_i) * np.log2(1 / (K_i * (Eu_i - nu * C_serv_i))), 0)
        return alpha_opt

# Initialize simulation
sim = EnergyOffloadingSimulation()
# Compute optimal alpha values for an arbitrary nu
optimal_alphas = sim.compute_optimal_alpha(nu=1e-5)
optimal_alphas[:10]  # Display first 10 results


###### Motion #########3
import numpy as np
import matplotlib.pyplot as plt

class MobileOffloadingSimulator:
    def __init__(self, num_users=50, area_radius=800, dt=0.1, total_time=10, seed=42):
        # System parameters
        self.N = num_users
        self.radius = area_radius
        self.dt = dt
        self.total_time = total_time
        self.steps = int(total_time / dt)
        # Offloading model parameters
        self.L = 10
        self.M = 60
        self.T = 0.02
        self.eta_i = 100
        self.epsilon_i = 5e-6
        self.N0 = 1e-9
        self.G = 1e-3
        self.d0 = 200
        self.NRB = 50
        self.B = self.NRB * 180e3
        self.eta_s = 1
        
        np.random.seed(seed)
        # Initialize random positions in circle
        angles = np.random.uniform(0, 2*np.pi, self.N)
        radii = np.sqrt(np.random.uniform(0, self.radius**2, self.N))
        self.positions = np.vstack([radii*np.cos(angles), radii*np.sin(angles)]).T
        
        # Random velocities (magnitude, direction)
        speeds = np.random.uniform(1, 5, self.N)  # m/s
        dirs = np.random.uniform(0, 2*np.pi, self.N)
        self.velocities = np.vstack([speeds*np.cos(dirs), speeds*np.sin(dirs)]).T
        
        # Data volumes fixed per user
        self.D = np.random.uniform(1e4, 1e6, self.N)
        
    def step(self):
        # Move users, reflect at boundary
        new_pos = self.positions + self.velocities * self.dt
        dist = np.linalg.norm(new_pos, axis=1)
        outside = dist > self.radius
        # Reflect velocities for those outside
        self.velocities[outside] *= -1
        self.positions += self.velocities * self.dt
        
    def compute_alpha(self, nu, beta):
        dists = np.linalg.norm(self.positions, axis=1)
        Eu = self.L * self.epsilon_i * self.eta_i * self.M
        K = (np.log(2) * self.N0 * self.D / self.G) * (dists/self.d0)**beta
        r = self.D / (self.B * self.T)
        alpha = (1/r) * np.log2(1/(K*(Eu - nu*self.eta_s*self.M)))
        return np.clip(alpha, 0, 1)
    
    def compute_energies(self, alpha, beta):
        dists = np.linalg.norm(self.positions, axis=1)
        Eu = (1 - alpha) * self.L * self.epsilon_i * self.eta_i * self.M
        factor = (2**(alpha*self.D/(self.B*self.T)) - 1)
        P_tr = factor*self.N0*self.B/self.G*(dists/self.d0)**beta
        Etr = P_tr * self.T
        return Eu, Etr
    
    def run(self, nu=1e-5, beta=3.0):
        avg_alpha = np.zeros(self.steps)
        total_energy = np.zeros(self.steps)
        for t in range(self.steps):
            alpha = self.compute_alpha(nu, beta)
            Eu, Etr = self.compute_energies(alpha, beta)
            avg_alpha[t] = np.mean(alpha)
            total_energy[t] = np.sum(Eu + Etr)
            self.step()
        time = np.arange(self.steps) * self.dt
        return time, avg_alpha, total_energy

# Run simulation
sim = MobileOffloadingSimulator()
time, avg_alpha, total_energy = sim.run(nu=1e-5, beta=3.0)

# Plot results
plt.figure(figsize=(12, 5))

plt.subplot(1,2,1)
plt.plot(time, avg_alpha, '-o', markersize=3)
plt.xlabel('Time (s)')
plt.ylabel('Average Offload Share (α)')
plt.title('Offload Share over Time')

plt.subplot(1,2,2)
plt.plot(time, total_energy, '-o', markersize=3)
plt.xlabel('Time (s)')
plt.ylabel('Total Energy (mJ)')
plt.title('Total Energy over Time')

plt.tight_layout()
plt.show()


############## dynamic pathloss ####################3
import numpy as np
import matplotlib.pyplot as plt

class DynamicPathlossOffloadingSimulator:
    def __init__(self, num_users=50, area_radius=800, dt=0.1, total_time=10, seed=42):
        # System parameters
        self.N = num_users
        self.radius = area_radius
        self.dt = dt
        self.total_time = total_time
        self.steps = int(total_time / dt)
        # Offloading model parameters
        self.L = 10
        self.M = 60
        self.T = 0.02
        self.eta_i = 100
        self.epsilon_i = 5e-6
        self.N0 = 1e-9
        self.G = 1e-3
        self.d0 = 200
        self.NRB = 50
        self.B = self.NRB * 180e3
        self.eta_s = 1
        
        np.random.seed(seed)
        # Initialize random positions within circle
        angles = np.random.uniform(0, 2*np.pi, self.N)
        radii = np.sqrt(np.random.uniform(0, self.radius**2, self.N))
        self.positions = np.vstack([radii*np.cos(angles), radii*np.sin(angles)]).T
        
        # Random velocities
        speeds = np.random.uniform(1, 5, self.N)
        dirs = np.random.uniform(0, 2*np.pi, self.N)
        self.velocities = np.vstack([speeds*np.cos(dirs), speeds*np.sin(dirs)]).T
        
        # Fixed data volumes
        self.D = np.random.uniform(1e4, 1e6, self.N)
        
    def dynamic_beta(self, distances):
        """Beta grows from 2 (at center) to 4 (at boundary), plus small log-normal shadowing."""
        base_beta = 2 + 2 * (distances / self.radius)
        # add small log-normal shadowing noise
        shadow = np.random.lognormal(mean=0, sigma=0.1, size=self.N) - 1
        return np.clip(base_beta + shadow, 2, 5)  # cap to [2,5]
        
    def step(self):
        # Move and reflect at boundary
        new_pos = self.positions + self.velocities * self.dt
        dist = np.linalg.norm(new_pos, axis=1)
        outside = dist > self.radius
        self.velocities[outside] *= -1
        self.positions += self.velocities * self.dt
        
    def compute_alpha(self, nu, beta):
        distances = np.linalg.norm(self.positions, axis=1)
        Eu = self.L * self.epsilon_i * self.eta_i * self.M
        K = (np.log(2) * self.N0 * self.D / self.G) * (distances/self.d0)**beta
        r = self.D / (self.B * self.T)
        alpha_unclipped = (1 / r) * np.log2(1 / (K * (Eu - nu * self.eta_s * self.M)))
        return np.clip(alpha_unclipped, 0, 1)
    
    def compute_energies(self, alpha, beta):
        distances = np.linalg.norm(self.positions, axis=1)
        Eu = (1 - alpha) * self.L * self.epsilon_i * self.eta_i * self.M
        factor = (2**(alpha * self.D / (self.B * self.T)) - 1)
        P_tr = factor * self.N0 * self.B / self.G * (distances/self.d0)**beta
        Etr = P_tr * self.T
        return Eu, Etr
    
    def run(self, nu=1e-5):
        time = np.arange(self.steps) * self.dt
        avg_alpha = np.zeros(self.steps)
        total_energy = np.zeros(self.steps)
        avg_beta = np.zeros(self.steps)
        
        for t in range(self.steps):
            distances = np.linalg.norm(self.positions, axis=1)
            beta_t = self.dynamic_beta(distances)
            alpha = self.compute_alpha(nu, beta_t)
            Eu, Etr = self.compute_energies(alpha, beta_t)
            
            avg_beta[t] = np.mean(beta_t)
            avg_alpha[t] = np.mean(alpha)
            total_energy[t] = np.sum(Eu + Etr)
            self.step()
        
        # Plot dynamic beta, alpha, energy
        fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        axs[0].plot(time, avg_beta, '-o', markersize=2)
        axs[0].set_ylabel('Avg Pathloss Exponent (β)')
        axs[0].set_title('Dynamic β over Time')
        axs[0].grid(True)
        
        axs[1].plot(time, avg_alpha, '-o', markersize=2)
        axs[1].set_ylabel('Avg Offload (α)')
        axs[1].set_title('Offload Share over Time')
        axs[1].grid(True)
        
        axs[2].plot(time, total_energy, '-o', markersize=2)
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Total Energy (mJ)')
        axs[2].set_title('Total Energy over Time')
        axs[2].grid(True)
        
        plt.tight_layout()
        plt.show()

# Run dynamic pathloss simulation
sim = DynamicPathlossOffloadingSimulator()
sim.run(nu=1e-5)


########## adjust the shadowing variance, velocity distribution, or introduce additional effects   ###################
'''
    Adjustable shadowing variance (shadow_sigma), now set to 0.5.
    Custom velocity range (speed_range=(0.1,10.0)), covering pedestrian to vehicular speeds.
    Small-scale Rayleigh fading in the transmission power.
    Dynamic β based on distance plus shadowing.

The resulting plots show spikes in energy when fading and shadowing peak, and more pronounced fluctuations in offload share
'''

import numpy as np
import matplotlib.pyplot as plt

class EnhancedDynamicSimulator:
    def __init__(self,
                 num_users=50,
                 area_radius=800,
                 dt=0.1,
                 total_time=10,
                 shadow_sigma=0.2,
                 speed_range=(0.5, 3.0),
                 shadow_mean=0,
                 seed=42):
        # Simulation parameters
        self.N = num_users
        self.radius = area_radius
        self.dt = dt
        self.steps = int(total_time / dt)
        # Model parameters
        self.L, self.M, self.T = 10, 60, 0.02
        self.eta_i, self.epsilon_i = 100, 5e-6
        self.N0, self.G, self.d0 = 1e-9, 1e-3, 200
        self.NRB, self.B = 50, 50 * 180e3
        self.eta_s = 1
        # Mobility & shadowing parameters
        self.shadow_sigma = shadow_sigma
        self.shadow_mean = shadow_mean
        self.min_speed, self.max_speed = speed_range
        np.random.seed(seed)
        # Initialize positions
        angles = np.random.uniform(0, 2*np.pi, self.N)
        radii = np.sqrt(np.random.uniform(0, self.radius**2, self.N))
        self.positions = np.vstack([radii*np.cos(angles), radii*np.sin(angles)]).T
        # Initialize velocities
        speeds = np.random.uniform(self.min_speed, self.max_speed, self.N)
        dirs = np.random.uniform(0, 2*np.pi, self.N)
        self.velocities = np.vstack([speeds*np.cos(dirs), speeds*np.sin(dirs)]).T
        # Data volumes
        self.D = np.random.uniform(1e4, 1e6, self.N)

    def dynamic_beta(self, distances):
        # Linear distance-based beta plus log-normal shadowing
        base_beta = 2 + 2 * (distances / self.radius)
        shadow = np.random.lognormal(mean=self.shadow_mean,
                                     sigma=self.shadow_sigma,
                                     size=self.N) - 1
        return np.clip(base_beta + shadow, 2, 5)

    def step(self):
        new_pos = self.positions + self.velocities * self.dt
        dist = np.linalg.norm(new_pos, axis=1)
        outside = dist > self.radius
        self.velocities[outside] *= -1
        self.positions += self.velocities * self.dt

    def compute_alpha(self, nu, beta):
        d = np.linalg.norm(self.positions, axis=1)
        Eu = self.L * self.epsilon_i * self.eta_i * self.M
        K = (np.log(2) * self.N0 * self.D / self.G) * (d/self.d0)**beta
        r = self.D / (self.B * self.T)
        alpha = (1/r) * np.log2(1 / (K * (Eu - nu * self.eta_s * self.M)))
        return np.clip(alpha, 0, 1)

    def compute_energies(self, alpha, beta):
        d = np.linalg.norm(self.positions, axis=1)
        Eu = (1 - alpha) * self.L * self.epsilon_i * self.eta_i * self.M
        factor = (2**(alpha * self.D / (self.B * self.T)) - 1)
        # Add small-scale Rayleigh fading factor
        fading = np.random.rayleigh(scale=1.0, size=self.N)
        P_tr = factor * self.N0 * self.B / self.G * (d/self.d0)**beta / fading
        Etr = P_tr * self.T
        return Eu, Etr

    def run(self, nu=1e-5):
        time = np.arange(self.steps) * self.dt
        avg_beta = np.zeros(self.steps)
        avg_alpha = np.zeros(self.steps)
        total_energy = np.zeros(self.steps)

        for t in range(self.steps):
            d = np.linalg.norm(self.positions, axis=1)
            beta = self.dynamic_beta(d)
            alpha = self.compute_alpha(nu, beta)
            Eu, Etr = self.compute_energies(alpha, beta)

            avg_beta[t] = np.mean(beta)
            avg_alpha[t] = np.mean(alpha)
            total_energy[t] = np.sum(Eu + Etr)
            
            self.step()

        # Plot
        fig, axs = plt.subplots(3,1, figsize=(8,10), sharex=True)
        axs[0].plot(time, avg_beta)
        axs[0].set_ylabel('Avg β')
        axs[0].grid(True)

        axs[1].plot(time, avg_alpha)
        axs[1].set_ylabel('Avg α')
        axs[1].grid(True)

        axs[2].plot(time, total_energy)
        axs[2].set_ylabel('Total Energy (mJ)')
        axs[2].set_xlabel('Time (s)')
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()


# Example: fine-tuned simulation
sim = EnhancedDynamicSimulator(shadow_sigma=0.5, speed_range=(0.1, 10.0))
sim.run(nu=2e-5)







