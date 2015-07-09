%% Handles the data aqcuisition for the hand eye calibration.
% A given number of calibration frames is acquired at poses within a
% maximum translation and a maximum rotation around a random axis.

%% Definitions
    % Network settrings
    camIP = '134.28.45.63'; % camera ip
    camPort = 3000; % camera port   
    timeout = 3000; % timeout for tcp reads
    
    % Camera settings
    camLocator = 'stylusRNM'; % locator name

    % Output settings
    camHTMFile = 'Input\needleTipLocator2.csv'; % output file for robot joints

%% Initialization
    % Connect to CamBarServer, if necessary
    if (~(exist('camSocket', 'var')))
        [camSocket, camInStream, camOutStream] = initCam(camIP, camPort, timeout);
        pause(1);
        disp('Initialized camera socket.');
    end;    
    
    % Register locator
    message = loadLocator(camSocket, camInStream, camOutStream, camLocator);
    disp(sprintf('Answer for: LoadLocator %s: %s', camLocator, message));
    
% get cam sample
    [camHTM, timestamp, isVisible, message] = getLocatorTransformMatrix(camSocket, camInStream, camOutStream, camLocator);
    camHTM(1:3,4)=camHTM(1:3,4);
%     camPoses(:,:,camCount) = camHTM;
%     camCount = camCount + 1;
    disp(camHTM);
    tumorPos = camHTM(1:3,4);
    
%% Save files
     csvwrite(camHTMFile, tumorPos);
     save('Calibration\Data\usTest.mat', 'camHTM');