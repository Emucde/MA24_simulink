function merged_cell = merge_cell_arrays(input_cells)
% Function to merge a cell array into a single row vector
% Description:
%   This function takes a cell array (`input_cells`) containing elements
%   of different sizes and merges them into a single 1xn cell array
%   (`merged_cell`), where n is the sum of all elements in the input cells.
%   Each element in the output cell array is a column vector.
%
% Inputs:
%   input_cells: A cell array containing elements of any size.
%
% Outputs:
%   merged_cell: A 1xn cell array where n is the sum of all elements
%                in the input cells. Each element is a column vector.

  % Reshape each element in the cell array to a column vector
  temp = cellfun(@(x) x(:)', input_cells, 'UniformOutput', false);

  % Concatenate all column vectors into a single cell array
  merged_cell = {[temp{:}]};
end