clc;clear;
parameters_7dof;

overwrite_offline_traj_forced_extern = true;
dont_clear = true;


for i = 1175:100000
    % Erstellen Sie eindeutige Namen für die Textdatei und das Video
    current_time = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    textFileName = sprintf('videos/max2_simulation_%05d_%s.txt', i, char(current_time));
    videoFileName = sprintf('videos/max2_simulation_%05d_%s', i, char(current_time));
    
    % Starten Sie die Aufzeichnung des Command Window
    diary(textFileName);
    
    % Führen Sie die Simulation aus
    sim('sim_discrete_7dof');
    
    % Beenden Sie die Aufzeichnung
    diary off;
    
    % Erstellen Sie das Video
    smwritevideo("sim_discrete_7dof", videoFileName, "PlaybackSpeedRatio", 5, "FrameRate", 10, "VideoFormat", "motion jpeg avi");

    pause(1);
    
    % Optional: Fortschrittsanzeige
    if mod(i, 100) == 0
        fprintf('Fortschritt: %d von 10000 Durchläufen abgeschlossen\n', i);
    end
end

fprintf('Alle 10000 Durchläufe abgeschlossen.\n');