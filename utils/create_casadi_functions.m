% CREATE CASADI FUNCTIONS FROM MAPLE FUNCTIONS

fun_path = "maple_generated/2_dof_system/"; % slash at end necessary
fun_arr = {"inertia_matrix.m", "coriolis_matrix.m", ...
    "gravitational_forces.m", "inertia_matrix_inv.m", "hom_transform_endeffector.m"};

for i=1:length(fun_arr)
    dat_path = char(fun_path + fun_arr{i});
    old_dat_name_split = split(dat_path, "/");
    old_dat_name = old_dat_name_split{end}; % last is filename
    old_fun_name = old_dat_name(1:end-2); % skip ".m" of filename
    newdat = ""+dat_path(1:end-2)+"_casadi.m";
    
    % copy old matlab function to new casadi matlab function
    copyfile(dat_path,newdat, 'f');
    
    % get first and second row of function
    datei = fopen(newdat, 'r');
    first_row = fgetl(datei); % function header
    second_row = fgetl(datei); % matrix initialization string
    eval(second_row); % create matrix m from string
    [n_row, n_col] = size(m); % get matrix dimension
    fclose(datei);
    
    % create new header and casadi matrix replacement
    first_row_new = replace(first_row, old_fun_name, old_fun_name+"_casadi");
    second_row_new = "  m = casadi.MX("+n_row+","+n_col+");";
    
    str = readlines(newdat);
    str(1,:) = first_row_new;
    str(2,:) = second_row_new;
    writelines(str,newdat);
end