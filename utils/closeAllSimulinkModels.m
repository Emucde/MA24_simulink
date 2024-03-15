function closeAllSimulinkModels(folderPath)
% closes all Simulink models with the .slx extension in a given folder

    % Check if the folder path is valid
    if ~exist(folderPath, 'dir')
      error(['Error: Folder "', folderPath, '" does not exist.']);
    end
    
    % Get all files with the .slx extension in the folder
    files = dir(fullfile(folderPath, '*.slx'));
    
    % Loop through each file and close the Simulink model
    for name = {files.name}
      % Construct the full path to the file
      fullPath = fullfile(folderPath, name{:});
      
      % Close the Simulink model
      try
        close_system(fullPath);
      catch ME
        % Handle any errors that might occur during closing
        warning(['Error closing Simulink model: ', fullPath]);
      end
    end
    
    % Display a message indicating completion
    disp(['Successfully closed all Simulink models in folder: ', folderPath]);
end