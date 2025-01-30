% add e.g.
%   export casadi_path=/media/daten/Anwendungen/casadi-3.6.6-linux64-matlab2018b
% to ~/.bashrc
casadi_path = getenv('casadi_path'); 
if(isempty(casadi_path))
    [flag, output] = system('cat ~/.bashrc');
    path_cell = strsplit(output, 'casadi_path=');
    path_cell = path_cell{end};
    path_cell = strsplit(path_cell, '\n');
    casadi_path = path_cell{1};
end
addpath(casadi_path);
import casadi.*
% warning('TODO: auf casadi 3.6.5 umsteigen')

%script_path = matlab.desktop.editor.getActiveFilename; % Get the path of the current script
%parts = split(script_path, "/"); % Split the path into its parts

%% Construct the path to the directory where the file should be generated
%path_of_file = "";
%for i = 2:length(parts)-1
%    path_of_file = path_of_file + "/" + parts(i);
%end
%cd(path_of_file);

setenv('GUROBI_VERSION', '120'); % Gurobi mag keine Singularitäten...

lib_path = GlobalOptions.getCasadiPath();
inc_path = GlobalOptions.getCasadiIncludePath();