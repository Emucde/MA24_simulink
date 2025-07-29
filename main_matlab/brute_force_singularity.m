% This code creates a simulation of a singularity analysis for a 7-DOF robot.
% It initializes the robot parameters, sets up the simulation environment,
% and runs the simulation multiple times, checking the feasibility of the
% joint configurations.
% The results are saved in text files and videos, and the simulation is
% repeated for a specified number of iterations.

clc;clear;
parameters_7dof;

overwrite_offline_traj_forced_extern = true;
overwrite_init_guess = false;
dont_clear = true;


for i = 1:1
    % create unique names for the text file and video
    current_time = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    textFileName = sprintf('videos/%05d/max2_simulation_%05d_%s.txt', i, i, char(current_time));
    videoFileName = sprintf('videos/%05d/max2_simulation_%05d_%s', i, char(current_time));

    % create folder
    if ~exist('videos', 'dir')
        mkdir('videos');
    end

    % create folder %05d
    if ~exist(sprintf('videos/%05d', i), 'dir')
        mkdir(sprintf('videos/%05d', i));
    end
    
    % start diary to record the simulation output
    diary(textFileName);
    
    % run the simulation
    sim_out = sim('sim_discrete_7dof');
    q = sim_out.q;
    q_p = sim_out.q_p;
    q_pp = sim_out.q_pp;


    % Extract time and values
    values_q = q_signal.Values.Data;
    values_q_p = q_p_signal.Values.Data;

    % Check if q is feasible
    q_is_feasible = check_q_feasibility(values_q, param_robot.q_limit_lower, param_robot.q_limit_upper);

    %if(q_is_feasible)
        % Beenden Sie die Aufzeichnung
        diary off;
    
        % Erstellen Sie das Video
        smwritevideo("sim_discrete_7dof", videoFileName, "PlaybackSpeedRatio", 5, "FrameRate", 10, "VideoFormat", "motion jpeg avi");
    
        % copy all files from folder [s_fun_path, '/trajectory_data/'] into %05d folder
        copyfile([s_fun_path, '/trajectory_data/'], sprintf('videos/%05d', i));
    %else
        % remove the text file and video file if q is not feasible
        %delete(textFileName);
        %delete([videoFileName, '.avi']);

        % remove folder %05d
        %rmdir(sprintf('videos/%05d', i));
    %end
    

    pause(1);
    
    % Optional: Fortschrittsanzeige
    if mod(i, 100) == 0
        fprintf('Fortschritt: %d von 10000 Durchläufen abgeschlossen\n', i);
    end
end

fprintf('Alle 10000 Durchläufe abgeschlossen.\n');

function q_is_feasible = check_q_feasibility(q, q_limit_lower, q_limit_upper)
    % Check if q is feasible
    q_is_feasible = all(q >= q_limit_lower) && all(q <= q_limit_upper);
end