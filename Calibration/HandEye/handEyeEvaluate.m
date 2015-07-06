%% Evaluates the calibration
clear all;
close all;


%% Definitions
    % Input settings
    cam2MarkerFile = 'Calibration\Data\camHTMs.mat';
    base2EndeffectorFile = 'Calibration\Data\robHTMs.mat';

    % Output settings
    calibrationMatricesFile = 'Calibration\Data\handEyeData.mat';
    robotToCamFileCSV = 'Input\rob2cam.csv';
    
%% Initialization  
    % load files
    load(cam2MarkerFile);
    load(base2EndeffectorFile);
    
    Cam2Marker = camHTMs;
    Base2Endeffector = robHTMs;
    nMeasurements = size(Cam2Marker,3);
    nCalibration = nMeasurements;
    nTest = nMeasurements - nCalibration;

    % scale camera data (robot data is provided in m whereas camera data is provided in mm)
    scalingFactor = 1*10^(-3);
    for i = 1:nMeasurements;
        Cam2Marker(1:3,4,i) = Cam2Marker(1:3,4,i) * scalingFactor;       
        Base2Endeffector(1:3,4,i) = Base2Endeffector(1:3,4,i) * scalingFactor; 
    end

    % split data in calibration and test set
    MCalib = Base2Endeffector(:,:,1:nCalibration);
    NCalib = Cam2Marker(:,:,1:nCalibration);
    MTest = Base2Endeffector(:,:,(nCalibration+1):(nCalibration+nTest));
    NTest = Cam2Marker(:,:,(nCalibration+1):(nCalibration+nTest));

%% Evaluate quality with respect to number of calibration points
    % init buffers
    errorTranslations = zeros(length(MCalib), 1);
    errorRotations = zeros(length(MCalib), 1); 
    skip = 5;
    for i = skip:length(MCalib);
        [X,Y] = handEyeErnstTwo(MCalib(:,:,1:i), NCalib(:,:,1:i));

        errorTranslationsLocal = zeros(length(MCalib), 1);
        errorRotationsLocal = zeros(length(MCalib), 1);    
        
        % calculate mean error for current number of calibrations used
        for j=1:length(MCalib);
            err = inv(X)*inv(MCalib(:,:,j))*Y*NCalib(:,:,j);
            [U,S,V] = svd(err(1:3,1:3));
            err = [U*V' err(1:3,4); 0 0 0 1];
            r = vrrotmat2vec(err(1:3,1:3));
            errorRotationsLocal(j) = abs(r(4))*180/pi;
            errorTranslationsLocal(j) = norm(err(1:3,4));
        end

        errorTranslations(i) = mean(errorTranslationsLocal);
        errorRotations(i) = mean(errorRotationsLocal);
    end

%% Plot errors
    figure(1);
    
    % Translational error
    subplot(2,1,1);
    plot(skip:length(MCalib), (1/scalingFactor)*errorTranslations(skip:length(errorRotations)), 'bo');
    grid on;
    xlabel('Number of calibration frames');
    ylabel('Absolute translational error / mm');
    hold on;

    % Rotational error
    subplot(2,1,2);
    plot(skip:length(MCalib), errorRotations(skip:length(errorRotations)), 'bo');
    grid on;
    xlabel('Number of calibration frames');
    ylabel('Absolute rotational error / °');
    hold on;
    
%% Save files
    save(calibrationMatricesFile, 'X', 'Y');
    csvwrite(robotToCamFileCSV, Y);
