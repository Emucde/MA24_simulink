addpath('/media/daten/Anwendungen/casadi-3.6.4-linux64-matlab2018b/')
import casadi.*
warning('TODO: auf casadi 3.6.5 umsteigen')

%script_path = matlab.desktop.editor.getActiveFilename; % Get the path of the current script
%parts = split(script_path, "/"); % Split the path into its parts

%% Construct the path to the directory where the file should be generated
%path_of_file = "";
%for i = 2:length(parts)-1
%    path_of_file = path_of_file + "/" + parts(i);
%end
%cd(path_of_file);

lib_path = GlobalOptions.getCasadiPath();
inc_path = GlobalOptions.getCasadiIncludePath();