%% Handles the data aqcuisition for the hand eye calibration.
% A given number of calibration frames is acquired. The poses need to be
% established by manually moving the robot.

%% Definitions
    % Calibration settings
    maxAngle = 15*pi/180; % maximum angle of rotation in degrees
    maxTranslation = 0.08; % maxiumum translation in meters
    nMeasurements = 50; % number of measurements
  
    % Network settrings
    robotIP = '192.168.56.101'; % robot ip
    robotPort = 5005; % robot port    
    camIP = '127.0.0.1'; % camera ip
    camPort = 3000; % camera port   
    timeout = 3000; % timeout for tcp reads
    
    % Camera settings
    camLocator = 'NeedleGRP3'; % locator name
    
    % Robot settings
    robotSpeed = 5; % speed of robot
        
    % Input settings
    defaultJointsFile = 'Data\robStartJoints.mat';
    
    % Output settings
    robJointsFile = 'Data\robJoints.mat'; % output file for robot joints
    robHTMsFile = 'Data\robHTMs.mat'; % output file for robot HTMs
    camHTMsFile = 'Data\camHTMs.mat'; % output file for camera HTMs

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
    robJoints = zeros(6, nMeasurements);
    robHTMs = zeros(4,4, nMeasurements);
    camHTMs = zeros(4,4, nMeasurements);
   
    
%% acquisition loop    
    acquisitionCounter = 1;
    continueAqcuisition = '';
    while (~(strcmpi(continueAqcuisition,'n')))
        
        disp(sprintf('Measurement %d', acquisitionCounter));      
        
        if ((exist('robotSocket', 'var')))
            deinitRobot(robotSocket, robotInStream, robotOutStream);
            pause(0.5);
            disp('Initialized robot socket.');
        end;
        
        input('Please move robot to next position.','s');
        
        % Connect to RobServer, if necessary
        if (~(exist('robotSocket', 'var')))
            [robotSocket, robotInStream, robotOutStream] = initRobot(robotIP, robotPort, timeout);
            pause(0.5);
            disp('Initialized robot socket.');
        end;

        % get cam sample
        [T, timestamp, isVisible, message] = getLocatorTransformMatrix(camSocket, camInStream, camOutStream, camLocator);
        
        if (isVisible)
            camHTMs(:,:,acquisitionCounter) = T;
            disp(sprintf('\t%s', 'Got camera sample'));
        
            % get robot sample       
            robJoints(:,acquisitionCounter) = getJointPositions(robotSocket, robotInStream, robotOutStream );
            robHTMs(:,:,acquisitionCounter) = getHTM(robotSocket, robotInStream, robotOutStream );
            disp(sprintf('\t%s', 'Got robot sample'));
            acquisitionCounter = acquisitionCounter + 1;
        else
            disp(sprintf('\t%s', 'Sample skipped: locator invisible'));
        end
        
        continueAqcuisition = input('Continue acquisition (y/n)? ','s');
    end;

%% Save files
    save(robJointsFile, 'robJoints');
    save(robHTMsFile, 'robHTMs');
    save(camHTMsFile, 'camHTMs');