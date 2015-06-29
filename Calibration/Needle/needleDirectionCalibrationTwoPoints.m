%% Calibrate needle direction

%% Definitions   
    % Input settings
    tipCalibrationFile = 'tipCalibrationSynth.mat'; % output file for camera HTMs
    endCalibrationFile = 'endCalibrationSynth.mat'; % output file for camera HTMs
    needleTipHTMFile = 'needleTipHTMSynth.mat';
%     tipCalibrationFile = 'tipCalibrationSynth.mat'; % output file for camera HTMs
%     stylusHTMsFile = 'stylusHTMsSynth.mat';
%     needleTipHTMFile = 'needleTipHTMSynth.mat';
    
    % Output settings
    rob2needleHTMFile = 'cam2needleHTM.mat';
    rob2needleHTMsCSVFile = 'cam2needleHTM.csv';

%% Initialization
    % Load Files
    load(tipCalibrationFile);
    load(endCalibrationFile);
    load(needleTipHTMFile);
    needleTipPoint = (needleTipHTM(1:3,4)+pTipPivot)';
    needleEndPoint = (needleTipHTM(1:3,4)+pEndPivot)';
    
%% Work part
    
    % create HTM that has z axis aligned to needle direction
    zAxisUnnormed = needleTipPoint - needleEndPoint;
    zAxis = zAxisUnnormed/norm(zAxisUnnormed);
    axisAngle = vrrotvec([0 0 1], zAxis);
    rotMat = vrrotvec2mat(axisAngle);
    translation = pPivot;
    HTM = [rotMat [translation]; 0 0 0 1];
    
    
%% Plot line fit and needle
    otherPoint = needleTipPoint - zAxis*norm(zAxisUnnormed);
    figure();
    hold on;
    plot3([needleTipPoint(1) needleEndPoint(1)], [needleTipPoint(2) needleEndPoint(2)], [needleTipPoint(3) needleEndPoint(3)], 'ko-');
    stretch = 5;
    pr0 = [0 0 0 1]';
    xr0 = stretch * [1 0 0 0]';
    yr0 = stretch * [0 1 0 0]';
    zr0 = stretch * [0 0 1 0]';
    quiv = zeros(4,4);
    quiv(1,:) = HTM*pr0;
    quiv(2,:) = HTM*xr0;
    quiv(3,:) = HTM*yr0;
    quiv(4,:) = HTM*zr0;
    quiver3(quiv(1,1),quiv(1,2),quiv(1,3),quiv(2,1),quiv(2,2),quiv(2,3),'r');
    quiver3(quiv(1,1),quiv(1,2),quiv(1,3),quiv(3,1),quiv(3,2),quiv(3,3),'g');
    quiver3(quiv(1,1),quiv(1,2),quiv(1,3),quiv(4,1),quiv(4,2),quiv(4,3),'b');
    hold off;
    
%% Save Files
    save(rob2needleHTMFile, 'HTM');
    csvwrite(rob2needleHTMsCSVFile, HTM);
