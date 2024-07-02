% Ensure that Simulink is open!
open_simulink_on_start = true;
%if(~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && sum(strfind([getenv().keys{:}], 'VSCODE')) == 0)
% && ~isSimulinkStarted funktioniert einfach nicht.
if( ~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && desktop('-inuse') == 1) % in vscode inuse delivers 0 (running in background)
    tic
    fprintf(['open simulink model \"' simulink_main_model_name, '\" ...'])
    open_system([simulink_main_model_name '.slx'])
    fprintf([' finished! (Loading time: ' num2str(toc) ' s)\n']);
end