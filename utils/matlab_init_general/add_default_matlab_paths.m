% add default paths
%restoredefaultpath

% Default robot independent paths
folder_add_to_path = {
    'main_matlab'
    'utils/matlab_utils'
    'utils/matlab_utils/trajectory_utils'
    'utils/matlab_utils/spline_functions'
    'utils/matlab_init_general'
    'utils/utils_casadi'
    'maple/maple_generated/fr3_7dof'  % yes this too, because I need T matrix from it for trajectory generation
    'urdf_creation'
    'main_simulink'
};

% Add to path 1/2
for i = 1:length(folder_add_to_path)
    if ~contains(path, folder_add_to_path{i})
        addpath(genpath(['../../', folder_add_to_path{i}]));
    end
end