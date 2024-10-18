%addpath(genpath('../'));
%addpath(genpath('../../utils/matlab_init_general/'));
%addpath(genpath('../../utils/matlab_utils/'));
cd /home/rslstudent/Students/Emanuel/MA24_simulink/
addpath(genpath('main_matlab'));
addpath(genpath('main_franka/Controller'));
addpath(genpath('main_franka/franka_matlab_v0.3.1'));

dont_clear = 1;
parameters_7dof;

cd /home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/Controller

param_global.Ta = 1e-3;
n=7;
T_sim = 10;

bus_definitions;
robot_name = 'fr3_no_hand_6dof';
param_robot_init;


%% PD+ and CT Controller settings
param_ct_pdplus_control_init;

K_d_t = 3*diag([1 1 1]);
K_p_t = K_d_t^2/4;

K_d_r = 3*diag([1 1 1]);
K_p_r = K_d_r^2/4;

ctrl_param.ct.Kd1 = blkdiag(K_d_t, K_d_r);
ctrl_param.ct.Kp1 = blkdiag(K_p_t, K_p_r);

ctrl_param.pd.D_d = 3*eye(6);
ctrl_param.pd.K_d = 3*eye(6);

%% Load Trajectory

%load('../../s_functions/fr3_no_hand_6dof/trajectory_data/param_traj_data.mat');
param_traj_data.N = length(param_traj_data.t);

% TODO: robot model bus mit C

robot_ip = '172.16.10.2';
% restoredefaultpath
% addpath(genpath('./s_functions'));

if(~exist('init_franka_flag', 'var'))
    init_franka_matlab('0.13.0');
    init_franka_flag = true;
end

q_init = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]';
param_global.Ta = 1e-3;

% dämpfung über diagonalterme der massenmatrix

load('mpc_init_ref_test.mat');
% load('mpc_init_guess_test.mat');
load('param_MPC8_init_guess.mat');
init_guess_0 = param_MPC8_init_guess.init_guess(2,:);
load('mpc_init_weight_test.mat');

open_simulink_on_start = true;
%if(~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && sum(strfind([getenv().keys{:}], 'VSCODE')) == 0)
% && ~isSimulinkStarted funktioniert einfach nicht.
simulink_main_model_name = 'realtime_simu_franka_fr3';
if( ~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && desktop('-inuse') == 1) % in vscode inuse delivers 0 (running in background)
    tic
    fprintf(['open simulink model \"' simulink_main_model_name, '\" ...'])
    open_system([simulink_main_model_name '.slx'])
    fprintf([' finished! (Loading time: ' num2str(toc) ' s)\n']);
end

cd /home/rslstudent/Students/Emanuel/MA24_simulink/