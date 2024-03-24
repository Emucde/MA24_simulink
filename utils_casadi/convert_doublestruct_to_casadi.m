function casadi_struct = convert_doublestruct_to_casadi(data_struct)
  import casadi.*;
  field_names = fieldnames(data_struct);
  casadi_struct = struct();
  for i = 1:length(field_names)
    field_name = field_names{i};
    field_value = data_struct.(field_name);
    if isnumeric(field_value)
      % Convert numeric data to CasADi SX symbols
      casadi_var = SX.sym(field_name, size(field_value));
      casadi_struct.(field_name) = casadi_var;
    else
      % Handle non-numeric fields (optional: warning or error)
      warning(['Field "', field_name, '" is not numeric. Skipping conversion.']);
    end
  end
end