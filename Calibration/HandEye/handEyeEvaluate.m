%% Evaluates the calibration

%% Definitions
    % Calibration settings
    cam2MarkerFile = 'Data\Cam2Marker.mat';
    base2EndeffectorFile = 'Data\Base2Endeffector.mat';
    load(cam2MarkerFile);
    load(base2EndeffectorFile);
    
    nMeasurements = size(Cam2Marker,3);
    nCalibration = 50;
    nTest = nMeasurements - nCalibration;

%% Initialization  
    % scale camera data (robot data is provided in m whereas camera data is provided in mm)
    scalingFactor = 1*10^(-3);
    for i = 1:nMeasurements;
        Cam2Marker(1:3,4,i) = Cam2Marker(1:3,4,i) * scalingFactor;        
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
    skip = 3;
    for i = skip:length(MCalib);
        [X,Y] = handEyeErnst(MCalib(:,:,1:i), NCalib(:,:,1:i));

        errorTranslationsLocal = zeros(length(MCalib), 1);
        errorRotationsLocal = zeros(length(MCalib), 1);    
        
        % calculate mean error for current number of calibrations used
        for j=1:length(MCalib);
            all = inv(X)*inv(MCalib(:,:,j))*Y*NCalib(:,:,j);
            [U,S,V] = svd(all);
            Q= U*(V');
            r = vrrotmat2vec(Q(1:3,1:3));
            errorRotationsLocal(j) = abs(r(4)*180/pi);
            errorTranslationsLocal(j) = norm(Q(1:3,4));
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
    ylabel('Absolute rotational error / �');
    hold on;
