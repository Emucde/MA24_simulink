% Collect unique variable names from function inputs and outputs
if(~exist('unique_f_opt_input_map', 'var'))
    unique_f_opt_input_map = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

if(~exist('unique_f_opt_output_map', 'var'))
    unique_f_opt_output_map = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

% collect unique inputs in map
for i = 1:length(f_opt_input_cell)
    for j = 1:length(f_opt_input_cell{i})
        key = f_opt_input_cell{i}{j}.name;
        if ~isKey(unique_f_opt_input_map, key)
            unique_f_opt_input_map(key) = true;
        end
    end
end

% collect unique outputs in map
for i = 1:length(f_opt_output_cell)
    for j = 1:length(f_opt_output_cell{i})
        key = f_opt_output_cell{i}{j}.name;
        if ~isKey(unique_f_opt_output_map, key)
            unique_f_opt_output_map(key) = true;
        end
    end
end