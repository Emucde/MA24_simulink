function z_MX = sx_to_mx(z_SX, mode_str)
% Converts an SX variable to an MX variable with a similar name and structure
% mode_str == 'get_MX_sym_cell'
% mode_str == 'get_MX_name_cell'
    import casadi.*;

    if(isa(z_SX, 'casadi.MX'))
        error(['Error: input ', str(z_SX), ' is already MX']);
    end

    if numel(z_SX) == 1 % Scalar case
        z_name = z_SX.name;
    else % Matrix case
        %z0 = z_SX(1, 1);  % Access first element
        %z0_name = z0.name;
        z0_name = str(z_SX(1,1));
        z_name = z0_name(1:end-2); % Extract name without '_0'
    end

    if(strcmp(mode_str, 'get_MX_sym_cell'))
        z_MX = MX.sym(z_name, size(z_SX)); % Create MX variable with consistent name 
    elseif(strcmp(mode_str, 'get_MX_name_cell'))
        z_MX = z_name;
    else
        error(['Error: mode_str = ', num2str(mode_str), ' is invalid (can be get_MX_sym_cell | get_MX_name_cell']);
    end

end