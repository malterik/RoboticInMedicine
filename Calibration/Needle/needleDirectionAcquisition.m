%% Acquire data for needle calibration
% After each acquisition the script waits for the needle to be moved to a
% new pose.

%% Definitions
    % Calibration settings
    nMeasurements = 50; % number of measurements
  
    % Network settrings
    camIP = '134.28.45.63'; % camera ip
    camIP = '127.0.0.1';
    camPort = 3000; % camera port   
    timeout = 3000; % timeout for tcp reads
    
    % Camera settings
    needleLocator = 'NeedleAdapter3'; % needle adapter locator
    stylusLocator = 'stylus2'; % stylus locator
    
    % Output settings
    needleTipHTMFile = 'needleHTM.mat'
    stylusHTMsFile = 'stylusHTMs.mat'; % output file for camera HTMs

%% Initialization
    % Connect to CamBarServer, if necessary
    if (~(exist('camSocket', 'var')))
        [camSocket, camInStream, camOutStream] = initCam(camIP, camPort, timeout);
        pause(1);
        disp('Initialized camera socket.');
    end;    
    
    % Register locators
    message = loadLocator(camSocket, camInStream, camOutStream, needleLocator);
    disp(message);
    message = loadLocator(camSocket, camInStream, camOutStream, stylusLocator);
    disp(message);
    
%% acquire data
    % create data buffers
    stylusHTMs = zeros(4,4, nMeasurements);
   
%% acquire needle tip position
    continueAqcuisition = '';
    while (~(strcmpi(continueAqcuisition,'n')))
        disp(sprintf('Acquring needle tip position'));
        % get cam sample
        [T, timestamp, isVisible, message] = getLocatorTransformMatrix(camSocket, camInStream, camOutStream, needleLocator);
        if (isVisible)
            needleTipHTM = T;
            disp(sprintf('\t%s', 'Got needle tip'));
        else
            disp(sprintf('\t%s', 'Skipped: locator invisible'));
        end
        
        continueAqcuisition = input('Redo acquisition of needle tip (y/n)? ','s');
    end;
     

%% acquisition loop
    disp(sprintf('Acquring stylus positions'));
    acquisitionCounter = 1;
    continueAqcuisition = '';
    while (~(strcmpi(continueAqcuisition,'n')))
        
        disp(sprintf('Measurement %d', acquisitionCounter));

        % get cam sample
        [T, timestamp, isVisible, message] = getLocatorTransformMatrix(camSocket, camInStream, camOutStream, stylusLocator);
        
        if (isVisible)
            stylusHTMs(:,:,acquisitionCounter) = T;
            disp(sprintf('\t%s', 'Got camera sample'));
            acquisitionCounter = acquisitionCounter + 1;
        else
            disp(sprintf('\t%s', 'Sample skipped: locator invisible'));
        end
        
        continueAqcuisition = input('Continue acquisition (y/n)? ','s');
    end;

%% Save files
    save(stylusHTMsFile, 'stylusHTMs');
    save(needleTipHTMFile, 'needleTipHTM');