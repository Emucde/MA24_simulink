function [run_flag, cnt] = update_traj_cnt(state_traj, run_flag, cnt, N)
    % UPDATERUNSTATUS Updates the run status based on state trajectory and parameters
    %
    % Inputs:
    %   state_traj - Structure containing start, reset, and stop flags
    %   run_flag   - Current run flag status
    %   cnt        - Current count
    %   N          - Maximum count value
    %
    % Outputs:
    %   run_flag   - Updated run flag status
    %   cnt        - Updated count
    %
    % This function updates the run status and count based on the input state
    % trajectory flags (start, reset, stop) and the current run flag status.
    
    start_val = state_traj.start;
    reset_val = state_traj.reset; % not used
    stop_val = state_traj.stop;
    
    if(run_flag == 0)
        if(start_val == 1 && reset_val == 0 && stop_val == 0)
            run_flag = 1;
            if(cnt < N)
                cnt = cnt + 1;
            end
        elseif(reset_val == 1 && cnt >= N) % sonst springt er
            cnt = 1;
        end
    elseif(run_flag == 1)

        if(stop_val == 1)
            run_flag = 0;
        elseif(cnt < N)
            cnt = cnt+1;
        else
            run_flag = 0;
        end

    end   
end
    