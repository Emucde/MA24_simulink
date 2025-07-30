% This file initializes the CasADi library for use in MATLAB.
% It sets up the necessary paths and imports the CasADi functions.

% addpath('/media/daten/Anwendungen/casadi-3.6.4-linux64-matlab2018b/')
casadi_path=getenv('casadi_path');
if isempty(casadi_path)
    [status,cmdout] = system('cat ~/.bashrc');
    cpath_cell = strsplit(cmdout, 'casadi_path=');
    casadi_path = cpath_cell{end};
    casadi_path = casadi_path(1:end-1);
end
addpath(casadi_path);
import casadi.*

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