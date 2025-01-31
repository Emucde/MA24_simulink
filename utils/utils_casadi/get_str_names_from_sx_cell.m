% INPUTS: [u_opt_sol, xx_full_opt_sol, cost_values_sol{1:length(cost_vars_SX)}] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
% IN: refvals, init_guess, param
% refvals: mpc_parameter_inputs = {x_k, y_d};
% init_guess: init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
%                          = [mpc_opt_var_inputs; lam_mpc_opt_var_inputs; lam_g_init_guess_0(:)
% param: pp

% OUT: u*, init_guess_out, Ji
% u*: u_opt
% init_guess_out = [mpc_opt_var_inputs; lam_mpc_opt_var_inputs; lam_g_init_guess_0(:)
% Ji = cost_vars_names

%% INPUTS

refval_str_cell = sxcell2str(mpc_parameter_inputs);
optval_str_cell = sxcell2str(mpc_opt_var_inputs);

n_optval = length(optval_str_cell);
init_guess_str_cell = cell(1, 2*n_optval+1);
for i=1:n_optval
    init_guess_str_cell(i) = optval_str_cell(i);
    tmp = optval_str_cell{i};
    tmp.name = ['lambda_', tmp.name];
    init_guess_str_cell(i+n_optval) = {tmp};
end

g_cell = merge_cell_arrays(g, 'vector');
g_dim1 = length(g_cell);
g_dim2 = 1;
init_guess_str_cell{end} = struct('name', 'g', 'dim_str', ['{',num2str(g_dim1),'x',num2str(g_dim2),'}'], 'dim', [g_dim1 g_dim2]);

xx_dims = cell(1, n_optval);
icnt = 0;
for i=1:length(mpc_opt_var_inputs)
    xx_dims(i) = {struct('name', optval_str_cell{1}.name, ...
        'i_start', icnt, ...
        'i_end', icnt+numel(mpc_opt_var_inputs{i})-1)};
    icnt = icnt + numel(mpc_opt_var_inputs{1});
end

param_weight_str_cell = sxcell2str(struct2cell(pp)');

%% OUTPUTS

% u_opt_indices_new = reshape(u_opt_indices, n, []);
u_opt_indices_new = u_opt_indices(:);

u_opt_sx_vals = w(u_opt_indices_new);
u_opt_sx_cell = num2cell(u_opt_sx_vals, 1);

u_opt_str_cell = sxcell2str(u_opt_sx_cell);

n_u_opt = length(u_opt_str_cell);

% glaub es kann eh nur 1 sein... [TODO]
icnt = 0;
for i=1:n_u_opt
    tmp = u_opt_str_cell{i};
    c_idx_start = u_opt_indices_new(1,i)-1;
    for j=1:length(xx_dims)
        if(c_idx_start < xx_dims{j}.i_end)
            break; % only then are indices in c correct
        end
        if(j<length(xx_dims))
            icnt = xx_dims{j+1}.i_start;
        end
    end
    indx_start = num2str(u_opt_indices_new(1,i)-1-icnt);
    indx_end = num2str(u_opt_indices_new(end,i)-1-icnt);
    u_opt_str_cell{i}.name = 'u_opt';
end

init_guess_out_str_cell = init_guess_str_cell;
for i=1:length(init_guess_str_cell)
    init_guess_out_str_cell{i}.name = [init_guess_out_str_cell{i}.name,'_out'];
end

cost_fun_names_cell = strsplit(cost_vars_names(2:end-1), ', ');
n_costfun = length(cost_fun_names_cell);
costfun_str_cell = cell(1, n_costfun);
for i=1:n_costfun
    costfun_str_cell{i} = struct('name', cost_fun_names_cell{i}, 'dim_str', '(1x1)', 'dim', [1, 1]);
end

% INPUTS
refval_str = str_cell2str(refval_str_cell, 'reference_values =');
init_guess_str = str_cell2str(init_guess_str_cell, 'init_guess =');
param_weight_str = str_cell2str(param_weight_str_cell, 'param_weight =');

f_opt_input_cell = {refval_str_cell, init_guess_str_cell, param_weight_str_cell};
f_opt_input_str = {refval_str, init_guess_str, param_weight_str};

% OUTPUTS
% u_opt_str_cell{1}.name = [u_opt_str_cell{1}.name, '_opt'];
u_opt_str = str_cell2str(u_opt_str_cell, 'u_opt =');
init_guess_out_str = str_cell2str(init_guess_out_str_cell, 'init_guess =');

costfun_str_cell_new =  cell(1, n_costfun);
for i=1:n_costfun
    costfun_str_cell_new{i} = str_cell2str(costfun_str_cell(i), [costfun_str_cell{i}.name, ' =']);
end

f_opt_output_cell = {u_opt_str_cell, init_guess_out_str_cell, costfun_str_cell};
f_opt_output_str = [{u_opt_str}, {init_guess_out_str}, costfun_str_cell_new(:)'];

function str_cell = sxcell2str(current_input_cell)
    %does only works for u_ strings (SX arrays)
    nn_len = length(current_input_cell);
    str_cell = cell(1, nn_len);
    for i=1:nn_len
        sym_name = str(current_input_cell{1,i}(1));
        sym_len  = size(current_input_cell{i});
        last_underline_pos = find(sym_name == '_', 1, 'last');
        if(isempty(last_underline_pos) || isnan(str2double(sym_name(last_underline_pos+1:end))))
            sym_str_split = sym_name;
        else
            sym_str_split = sym_name(1:last_underline_pos-1);
        end
        str_cell{i} = struct('name', sym_str_split, ...
            'dim_str', ['(', num2str(sym_len(1)),'x', num2str(sym_len(2)), ')'], ...
            'dim', [sym_len(1), sym_len(2)]);
    end
end

function str_out = str_cell2str(current_input_cell, prestring)
    nn_len = length(current_input_cell);
    str_out = [prestring,' ['];
    for i=1:nn_len
        sym_name = current_input_cell{i}.name;
        sym_dim =  current_input_cell{i}.dim_str;
        str_out = [str_out, sym_name, sym_dim];
        if(i < nn_len)
            str_out = [str_out, ', '];
        end
    end
    str_out = [str_out, '] '];
end