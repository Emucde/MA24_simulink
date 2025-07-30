function casadi_struct = convert_doublestruct_to_casadi(data_struct, casadi_datatype_fun)
% convert_doublestruct_to_casadi converts a structure with numeric fields
% to a CasADi structure with symbolic variables. Used for converting the parameters
% of a CasADi MPC problem to a CasADi structure.
  arguments
    data_struct = struct;
    casadi_datatype_fun = @casadi.SX.sym; % default SX datatype
  end

  %import casadi.*;
  field_names = fieldnames(data_struct);
  casadi_struct = struct();
  for i = 1:length(field_names)
    field_name = field_names{i};
    field_value = data_struct.(field_name);
    if isnumeric(field_value)
      % Convert numeric data to CasADi SX symbols
      if(isdiag(field_value) && ~isvector(field_value))
        m = size(field_value, 1);
        casadi_var = diag(casadi_datatype_fun(field_name, m));
      elseif(isrow(field_value))
        casadi_var = casadi_datatype_fun(field_name, size(field_value))';
      else
        casadi_var = casadi_datatype_fun(field_name, size(field_value));
      end
      casadi_struct.(field_name) = casadi_var;
    else
      % Handle non-numeric fields (optional: warning or error)
      warning(['Field "', field_name, '" is not numeric. Skipping conversion.']);
    end
  end
end