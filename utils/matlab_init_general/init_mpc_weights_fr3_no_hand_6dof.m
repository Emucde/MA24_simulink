jsonText = fileread('./config_settings/casadi_mpc_weights_fr3_no_hand.json'); % Read JSON file as text
param_weight = jsondecode(jsonText);        % Convert JSON text to structure

%%%%%%%%%%%%%%%%%%% generate param MPC weights struct %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
names = fieldnames(param_weight)';
param_weight_init = struct;
for name=names
    mpc_name = name{1};
    temp = merge_cell_arrays(struct2cell(param_weight.(mpc_name))');
    param_weight_init.(mpc_name) = temp{1};
end