%% Acquire data for needle calibration
% After each acquisition the script waits for the needle to be moved to a
% new pose.

%% Definitions
    % Calibration settings
    nMeasurements = 50; % number of measurements
  
    % Network settrings
    camIP = '134.28.45.63'; % camera ip
    camPort = 3000; % camera port   
    timeout = 3000; % timeout for tcp reads
    
    % Camera settings
    camLocator = 'NeedleAdapter3'; % locator name  
    
    % Output settings
    camHTMsFile = 'needleEndHTMs.mat'; % output file for camera HTMs

%% Initialization
    % Connect to CamBarServer, if necessary
    if (~(exist('camSocket', 'var')))
        [camSocket, camInStream, camOutStream] = initCam(camIP, camPort, timeout);
        pause(1);
        disp('Initialized camera socket.');
    end;    
    
    % Register locator
    message = loadLocator(camSocket, camInStream, camOutStream, camLocator);
    disp(message);
    
%% acquire data
    % create data buffers
    camNeedleHTMs = zeros(4,4, nMeasurements);
   
    
%% acquisition loop    
    acquisitionCounter = 1;
    continueAqcuisition = '';
    while (~(strcmpi(continueAqcuisition,'n')))
        
        disp(sprintf('Measurement %d', acquisitionCounter));

        % get cam sample
        [T, timestamp, isVisible, message] = getLocatorTransformMatrix(camSocket, camInStream, camOutStream, camLocator);
        
        if (isVisible)
            camNeedleHTMs(:,:,acquisitionCounter) = T;
            disp(sprintf('\t%s', 'Got camera sample'));
            acquisitionCounter = acquisitionCounter + 1;
        else
            disp(sprintf('\t%s', 'Sample skipped: locator invisible'));
        end
        
        continueAqcuisition = input('Continue acquisition (y/n)? ','s');
    end;

%% Save files
    save(camHTMsFile, 'camNeedleHTMs');