%% Calibrate endeffector to needle tip transformation
% clear all;
% close all;

%% Definitions   
    show_plot = 1;

    % Input settings
    tipHTMsFile =  'Calibration\Data\needleTipHTMs.mat'; % contains HTMs of needle tips
    endHTMsFile =  'Calibration\Data\needleEndHTMs.mat'; % contains HTMs of needle ends
    HandEyeFile =  'Calibration\Data\handEyeData.mat'; % contains the hand eye calibration matrices
    
    % Output settings
    rob2NeedleHTMFileCSV = 'Input\rob2needle.csv'; 
    
%% Initialization
    % load files
    load(tipHTMsFile);
    load(endHTMsFile);
    load(HandEyeFile);

    % scale camera data to m
    needleTipHTMs(1:3,4,:) = needleTipHTMs(1:3,4,:) / 1000;
    needleEndHTMs(1:3,4,:) = needleEndHTMs(1:3,4,:) / 1000;

    % calibrate needle tip and end pivot vectors
    [ pCalTip, pPivotTip, errorTip ] = solveNC(needleTipHTMs, show_plot);
    [ pCalEnd, pPivotEnd, errorEnd ] = solveNC(needleEndHTMs, show_plot);

%% Create needle in camera coordinates    
    % create HTM for needle tip that has its z axis aligned to needle with
    % needle direction
    zAxis = pCalTip - pCalEnd;
    axisAngle = vrrotvec([0 0 1], zAxis);
    needleTipHTM_cam = [vrrotvec2mat(axisAngle) pCalTip; 0 0 0 1];
  
%% Transform needle to robot coordinates
    rob2needle = orth(X*needleTipHTM_cam);

%% Save files
    csvwrite(rob2NeedleHTMFileCSV, rob2needle);

