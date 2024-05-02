function str = plot_eigenvalues_controller_text(data, header_str_1, header_str_2)
%plot_eigenvalues_controller_text gGenerates a formatted string from the input data based on specific operations.
%   str = create_helper_text(data, header_str_1, header_str_2) generates a string from the input data based on specific operations.
%   The function converts the input matrix to a string with '|' separator in every third column and adds headers.
%
%   Inputs:
%   - data: Input matrix.
%   - header_str_1: String of the first header.
%   - header_str_2: String of the second header.
%
%   Output:
%   - str: The generated string from the input data.
%
%   Example:
%   str = plot_eigenvalues_controller_text(data, 'Header1', 'Header2');


    % Convert the matrix to a string with '|' separator in every third column
    str = '';
    col_len = size(data, 1);
    row_len = size(data, 2);
    for i = 1:col_len
        for j = 1:row_len
            if(j==1)
                if(i == 1)
                    str=[str 'λ+ | '];
                elseif(i==2)
                    str=[str 'λ- | '];
                end
            end
            char_col = max(arrayfun(@(i) numel(num2str(data(i,j), '%0.3g')), 1:col_len))+2;
            str = [str num2str(data(i,j), '%0.3g') repmat(' ', 1, char_col - numel(num2str(data(i,j), '%0.3g')))];
            if j == 6
                str = [str '| '];
            end
        end
        str = [str newline];
    end
    
    %header_str = '  max(y_pos_solution)                                       | max(y_soll)                                                 | max(y_pos_solution-y_pos_point)\n';
    pipe_pos = strfind(str(1,:), '|');
    header_str_1 = append('   | ', header_str_1);
    header_str_2 = append('| ', header_str_2);

    header_str = append(header_str_1, repmat(' ', 1, pipe_pos(2)-1-length(header_str_1)), header_str_2, '\n');
    str = [newline header_str str];
    fprintf(str);
end