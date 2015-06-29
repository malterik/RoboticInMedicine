%% Calibrate needle tip position

%% Definitions   
    % Input settings
    camHTMsFile = 'camNeedleHTMs.mat'; % output file for camera HTMs
    % Output settings
    tipCalibrationFile = 'tipCalibration.mat';
    
    
    load(camHTMsFile);
    
    [ pCal, pPivot ] = solveNC(camNeedleHTMs);    
    
    save(tipCalibrationFile, 'pPivot', 'pCal');