% This was a testing file for the MATLAB command listener.
% It listens for commands in a file and executes them in MATLAB.
% Make sure to set the masterdir environment variable before running.

cd(getenv("masterdir"));
run('./main_matlab/parameters_7dof');

clc;clear;

disp('MATLAB command listener started');
while true
    try
        if exist('commands.txt', 'file')
           command = fileread('commands.txt');
           eval(command);
           delete('commands.txt');
        else
            command = '';
        end
        pause(0.1);
     catch
        disp(['not valid command:', command]);
        delete('commands.txt');
     end
end