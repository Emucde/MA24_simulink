function [output_names, param_mpc_source_selection] = get_mpc_param_list(param_casadi_fun_name, param_casadi_fun_struct, mpc_source_selection, iterate_all_mpc_sfunctions)

    if(mpc_source_selection == 5)
        param_casadi_fun_struct_list_temp = struct2cell(init_custom_mpc_list());
    else
        param_casadi_fun_struct_list_temp = struct2cell(param_casadi_fun_name);
    end

    if(mpc_source_selection == 1) % all mpcs
        param_casadi_fun_struct_list = param_casadi_fun_struct_list_temp;
    elseif(mpc_source_selection == 2) % only dyn mpc
        param_casadi_fun_struct_list = param_casadi_fun_struct_list_temp(cellfun(@(x) isempty(strfind(x.version, 'kin')), param_casadi_fun_struct_list_temp));
    elseif(mpc_source_selection == 3) % only kin mpc
        param_casadi_fun_struct_list = param_casadi_fun_struct_list_temp(cellfun(@(x) ~isempty(strfind(x.version, 'kin')), param_casadi_fun_struct_list_temp));
    elseif(mpc_source_selection == 4) % only selected mpc
        param_casadi_fun_struct_list = {param_casadi_fun_struct};
    elseif (mpc_source_selection == 5) % custom list
        param_casadi_fun_struct_list = param_casadi_fun_struct_list_temp;
    else
        error('mpc_source_selection not defined');
    end

    output_names = cellfun(@(x) x.name, param_casadi_fun_struct_list, 'UniformOutput', false);
    param_mpc_source_selection = param_casadi_fun_struct_list;

end