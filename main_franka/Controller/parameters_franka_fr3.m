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

param_global.Ta = 1e-3; %nötig?
n=7;
T_sim = 10;

bus_definitions;
robot_name = 'fr3_no_hand_6dof';
param_robot_init;

robot_ip = '172.16.10.2';
% restoredefaultpath
% addpath(genpath('./s_functions'));

if(~exist('init_franka_flag', 'var'))
    init_franka_matlab('0.13.0');
    init_franka_flag = true;
end

q_init = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]';

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


% Reset all inputs so that it must not recompile

state_traj_const_names = {'Start Trajectory', 'Reset Trajectory', 'Stop Trajectory', 'home', 'home mode'};
for i = 1:length(state_traj_const_names)
    set_param(['realtime_simu_franka_fr3/', state_traj_const_names{i}], 'Value', '0')
end
set_param('realtime_simu_franka_fr3/trajectory selector', 'Value', '1')