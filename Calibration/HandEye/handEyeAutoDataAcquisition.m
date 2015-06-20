%% Handles the data aqcuisition for the hand eye calibration.
% A given number of calibration frames is acquired at poses within a
% maximum translation and a maximum rotation around a random axis.

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
    camLocator = 'RobAdapter'; % locator name
    
    % Robot settings
    robotSpeed = 100; % speed of robot
        
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

    % Connect to RobServer, if necessary
    if (~(exist('robotSocket', 'var')))
        [robotSocket, robotInStream, robotOutStream] = initRobot(robotIP, robotPort, timeout);
        pause(1);
        disp('Initialized robot socket.');
    end;
    
    % Register locator
    message = loadLocator(camSocket, camInStream, camOutStream, camLocator);
    disp(sprintf('Answer for: LoadLocator %s: %s', camLocator, message));
    
    % Set robot speed
    cmd = sprintf('SetSpeed %d', robotSpeed);
    message = getAnswerFromServer(robotSocket, robotInStream, robotOutStream, sprintf('SetSpeed %d', robotSpeed));
    disp(sprintf('Answer for: %s: %s', cmd, message));
    
    % Load starting position for calibration
    load(defaultJointsFile);
    defaultHTM = directKinematics(defaultJoints*pi/180);  
    
    
%% reach starting position
    [jointPositions,configuration] = inverseKinematicsAndPlanningUR5(defaultHTM);
    jointPositions = jointPositions*180/pi;
    jp = jointPositions(:,1);
    message = movePTPJoints(robotSocket, robotInStream, robotOutStream, defaultJoints);
    waitForCompletion(robotSocket, robotInStream, robotOutStream, 'Waiting for robot to reach default calibration pose', 1);
    
%% acquire data
    % create data buffers
    robJoints = zeros(6, nMeasurements);
    robHTMs = zeros(4,4, nMeasurements);
    camHTMs = zeros(4,4, nMeasurements);
    
    % acquisition loop
    acquisitionCounter = 1;
    while (acquisitionCounter <= 50);
        disp(sprintf('Measurement %d', acquisitionCounter));

        % create random calibration pose   
        [calibrationPosition, calibrationPositionJoints] = createNearbyHTM(defaultJoints', defaultHTM, maxAngle, maxTranslation);

        % reach new calibration position   
        message = movePTPJoints(robotSocket, robotInStream, robotOutStream, calibrationPositionJoints);

        % wait until position is reached
        waitForCompletion(robotSocket, robotInStream, robotOutStream, 'Waiting for robot to reach new calibration pose', 1);

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
    end

%% Save files
    save(robJointsFile, 'robJoints');
    save(robHTMsFile, 'robHTMs');
    save(camHTMsFile, 'camHTMs');