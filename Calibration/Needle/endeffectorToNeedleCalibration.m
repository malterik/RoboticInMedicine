%% Calibrate endeffector to needle tip transformation
clear all;
close all;

%% Definitions   
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
    [ pCalTip, pPivotTip ] = solveNC(needleTipHTMs);
    [ pCalEnd, pPivotEnd ] = solveNC(needleEndHTMs);
    
    % create cam and rob pose from hand eye calibration matrices
    camPose = eye(4);
    robPose = Y*camPose*inv(X);
    [U,S,V] = svd(robPose(1:3,1:3));
    robPose = [U*V' robPose(1:3,4); 0 0 0 1];

%% Create needle in camera coordinates    
    % create HTM for needle tip that has its z axis aligned to needle with
    % needle direction
    zAxis = pCalTip - pCalEnd;
    axisAngle = vrrotvec([0 0 1], zAxis);
    needleTipHTM_cam = [vrrotvec2mat(axisAngle) pCalTip; 0 0 0 1];
    needleEndHTM_cam = [vrrotvec2mat(axisAngle) pCalEnd; 0 0 0 1];
  
%% Transform needle to robot coordinates
    % calculate needle tip HTM and the end point in robot coordinates
    needleTipHTM_rob = Y*needleTipHTM_cam;
    [U,S,V] = svd(needleTipHTM_rob(1:3,1:3));
    needleTipHTM_rob = [U*V' needleTipHTM_rob(1:3,4); 0 0 0 1];    
    needleEndPoint_rob = Y*[pCalEnd;1];
    needleEndPoint_rob = needleEndPoint_rob(1:3); 
    
     
    % create HTM for needle tip that has its z axis aligned to needle with
    % needle direction
    zAxis = needleTipHTM_rob(1:3,4) - needleEndPoint_rob;
    axisAngle = vrrotvec([0 0 1], zAxis);
    needleTipHTM_rob = [vrrotvec2mat(axisAngle) needleTipHTM_rob(1:3,4); 0 0 0 1];  
   
    % Calculate endeffector to needle transformation
    rob2needle = inv(robPose)*needleTipHTM_rob

%% Save files
    csvwrite(rob2NeedleHTMFileCSV, rob2needle);
    csvwrite('Calibration\Data\needleLength.csv', norm(needleTipHTM_cam(1:3,4)-needleEndHTM_cam(1:3,4)));