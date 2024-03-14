folder_path = './MPC_shared_subsystems';

% Get all files in the folder
files = dir(fullfile(folder_path, '*.slx'));

% Loop through each file
for name = {files.name}
    close_system(fullfile(folder_path, name{:}));
end