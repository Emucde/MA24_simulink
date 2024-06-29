function closeAllSimulinkModels(folderPath)
  % closes all Simulink models with the .slx extension in a given folder and its subfolders
  
      % Check if the folder path is valid
      if ~exist(folderPath, 'dir')
        error(['Error: Folder "', folderPath, '" does not exist.']);
      end
      
      % Get all files and folders in the given folder
      contents = dir(folderPath);
      
      % Loop through each item in the folder
      for item = contents'
          % Check if the item is a folder (excluding . and ..)
          if item.isdir && ~strcmp(item.name, '.') && ~strcmp(item.name, '..')
              % Recursively call the function for the subfolder
              closeAllSimulinkModels(fullfile(folderPath, item.name));
          elseif endsWith(item.name, '.slx')
              % Construct the full path to the Simulink model
              fullPath = fullfile(folderPath, item.name);
              
              % Close the Simulink model
              try
                  %disp(['Closing Simulink model: ', fullPath])
                  close_system(fullPath, 0);
              catch ME
                  % Handle any errors that might occur during closing
                  warning(['Error closing Simulink model: ', fullPath]);
              end
          end
      end
  end