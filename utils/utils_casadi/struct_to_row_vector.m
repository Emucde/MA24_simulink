function row_vector = struct_to_row_vector(data_struct)
% struct_to_row_vector - Converts a structure or cell array of structures to a row vector.
% If the input is a structure, it flattens all fields into a single row vector.
% If the input is a cell array of structures, it concatenates the row vectors of each
% structure into a single row vector.
%% Usage:
%   row_vector = struct_to_row_vector(data_struct)
  if iscell(data_struct)
    row_vector = [cellfun(@struct_to_row_vector, data_struct)];
  elseif isstruct(data_struct)
    row_vector = [struct_to_row_vector_all(data_struct)];
  else
    row_vector = data_struct(:)';
  end
end

function row_vector = struct_to_row_vector_all(data_struct)
  field_names = fieldnames(data_struct);
  field_data = cell(1, length(field_names));
  for i = 1:length(field_names)
    field_value = data_struct.(field_names{i});
    field_data{i} = field_value(:)';
  end
  row_vector = [field_data{:}];
end