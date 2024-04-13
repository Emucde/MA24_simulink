function merged_cell = merge_cell_arrays(input_cells, return_type)
arguments
  input_cells = {};
  return_type = "cell"; % default SX datatype
end
% Function to merge a cell array into a single row vector
% Description:
%   This function takes a cell array (`input_cells`) containing elements
%   of different sizes and merges them into a single 1xn cell array
%   (`merged_cell`), where n is the sum of all elements in the input cells.
%   Each element in the output cell array is a column vector.
%
% Inputs:
%   input_cells: A cell array containing elements of any size.
%   return_type: returns merged cell as 1d cell or 1d vector.
%
% Outputs:
%   merged_cell: A 1xn cell array where n is the sum of all elements
%                in the input cells. Each element is a column vector.

  % Reshape each element in the cell array to a column vector
  temp = cellfun(@(x) x(:)', input_cells, 'UniformOutput', false);

  % Concatenate all column vectors into a single cell array
  if(strcmp(return_type, "cell"))
    merged_cell = {[temp{:}]};
  elseif(strcmp(return_type, "vector"))
    merged_cell = [temp{:}];
  else
    error('return type have to be "cell" or "vector"!');
  end
end