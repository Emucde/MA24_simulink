% comments all unused mpcs out and only used mpc block in.

if(bdIsLoaded(simulink_main_model_name))
    % get selected mpc version
    mpc_list = get_param('sim_discrete_7dof/controller combo box', 'States');
    selected_mpc_index = str2double(get_param('sim_discrete_7dof/controller combo box', 'Value'));
    selected_mpc_label_name = mpc_list(selected_mpc_index).Label;
    names = strsplit(selected_mpc_label_name, ' ');
    selected_mpc_name = names{1};

    % get list of all blocks in controller subsystem
    controller_blocklist = get_param('sim_discrete_7dof/Simulation models/Controller Subsystem/', 'Blocks');
    
    % get Names of MPC subsystems
    mpc_subsys_list = controller_blocklist(cellfun(@(x) contains(x, 'MPC'), controller_blocklist));
    
    for i=1:length(mpc_subsys_list)
        mpc_subsys_string = [simulink_main_model_name, '/Simulation models/Controller Subsystem/', mpc_subsys_list{i}];
        mpc_sfun_string = [mpc_subsys_string, '/S-Function'];
        mpc_getdata_string = [simulink_main_model_name, '/Simulation models/Controller Subsystem/', mpc_subsys_list{i}, '/get data subsys/get data'];
        sfun_mpc_modules = get_param(mpc_sfun_string, 'SFunctionModules');

        comment_state = get_param(mpc_sfun_string, 'Commented');


        mpc_number_str_cell = strsplit(sfun_mpc_modules, 'MPC');
        mpc_number = str2double(mpc_number_str_cell{end}(1:end-1));

        if(special_comment_mode)
            if(mpc_number > 7 && mpc_number ~= 14)
                force_uncommenting = true;
            else
                force_uncommenting = false;
            end
        else
            force_uncommenting = false;
        end

        if(contains(mpc_subsys_list{i}, selected_mpc_name)  || force_uncommenting)
            if(strcmp(comment_state, 'on'))
                for j=1:5
                    try
                        % unsetting the module enforces reloading of the mpc!
                        get_param(mpc_sfun_string, 'SFunctionModules');
                        set_param(mpc_sfun_string, 'SFunctionModules', '''''');
                        
                        % Setting the same parameter again enforces loading of the mpc!
                        set_param(mpc_sfun_string, 'SFunctionModules', sfun_mpc_modules);
                        break;
                    end
                end
                set_param(mpc_sfun_string, 'Commented', 'off');
                set_param(mpc_getdata_string, 'Commented', 'off');
            end
        else
            if(strcmp(comment_state, 'off'))
                set_param(mpc_sfun_string, 'Commented', 'on');
                set_param(mpc_getdata_string, 'Commented', 'on');
            end
        end
    end
end