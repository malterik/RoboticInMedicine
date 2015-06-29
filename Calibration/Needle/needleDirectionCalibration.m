%% Calibrate needle direction

%% Definitions   
    % Input settings
    tipCalibrationFile = 'tipCalibration.mat'; % output file for camera HTMs
    stylusHTMsFile = 'stylusHTMs.mat';
    needleTipHTMFile = 'needleHTM.mat';
%     tipCalibrationFile = 'tipCalibrationSynth.mat'; % output file for camera HTMs
%     stylusHTMsFile = 'stylusHTMsSynth.mat';
%     needleTipHTMFile = 'needleTipHTMSynth.mat';
    
    % Output settings
    rob2needleHTMFile = 'cam2needleHTM.mat';
    rob2needleHTMsCSVFile = 'cam2needleHTM.csv';

%% Initialization
    % Load Files
    load(tipCalibrationFile);
    load(stylusHTMsFile);
    load(needleTipHTMFile);
    needleTipPoint = (needleTipHTM(1:3,4)+pPivot)';
    
%% Work part

    nMeasurements = size(stylusHTMs, 3);
    stylusTranslations = zeros(nMeasurements, 3);
    for i = 1:nMeasurements;
        stylusTranslations(i,:) = stylusHTMs(1:3,4,i);
    end
    
    [err,N,P] = fit_3D_data(stylusTranslations(:,1),stylusTranslations(:,2),stylusTranslations(:,3),'line','off','off');
    N = N';
    
    % make N point the right direction
    testPoint = stylusTranslations(1,:);
    testNorm = norm(needleTipPoint-testPoint);
    vecPlus = needleTipPoint + testNorm * N;
    vecMinus = needleTipPoint - testNorm * N;
    if (norm(vecPlus-testPoint) < norm(vecMinus-testPoint));
        N = (-1)*N;    
    end
 
    
    % create HTM that has z axis aligned to needle direction
    zAxis = N;
    axisAngle = vrrotvec([0 0 1], zAxis);
    rotMat = vrrotvec2mat(axisAngle);
    translation = pPivot;
    HTM = [rotMat [translation]; 0 0 0 1];
    
    
%% Plot line fit and needle
    otherPoint = needleTip - N*norm(dirVec);
    figure();
    hold on;
    plot3(stylusTranslations(:,1), stylusTranslations(:,2), stylusTranslations(:,3), 'bo');
    plot3([otherPoint(1) needleTip(1)], [otherPoint(2) needleTip(2)], [otherPoint(3) needleTip(3)], 'k-');
    plot3(needleTipPoint(1), needleTipPoint(2), needleTipPoint(3), 'kx', 'markerSize', 10);
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
