%% Handles the data aqcuisition for the hand eye calibration.
% A given number of calibration frames is acquired at poses within a
% maximum translation and a maximum rotation around a random axis.

%% Definitions
    % Calibration settings
    maxAngle = 8*pi/180; % maximum angle of rotation in degrees
    maxTranslation = 0.10; % maxiumum translation in meters
    nMeasurements = 30; % number of measurements
  
    % Network settrings
    robotIP = '134.28.45.95'; % robot ip
    robotPort = 5005; % robot port    
    camIP = '134.28.45.63'; % camera ip
    camPort = 3000; % camera port   
    timeout = 3000; % timeout for tcp reads
    
    % Camera settings
    camLocator = 'NeedleGRP4'; % locator name
    
    % Robot settings
    robotSpeed = 5; % speed of robot
        
    % Input settings
    defaultJointsFile = 'Calibration\Data\robStartJoints.mat';
    
    % Output settings
    robJointsFile = 'Calibration\Data\robJoints.mat'; % output file for robot joints
    robHTMsFile = 'Calibration\Data\robHTMs.mat'; % output file for robot HTMs
    camHTMsFile = 'Calibration\Data\camHTMs.mat'; % output file for camera HTMs

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
    while (acquisitionCounter <= nMeasurements);
        disp(sprintf('Measurement %d', acquisitionCounter));

        % create random calibration pose   
        [calibrationPosition, calibrationPositionJoints] = createNearbyHTM(defaultJoints', defaultHTM, maxAngle, maxTranslation);

        % reach new calibration position   
        message = movePTPJoints(robotSocket, robotInStream, robotOutStream, calibrationPositionJoints);

        % wait until position is reached
        waitForCompletion(robotSocket, robotInStream, robotOutStream, 'Waiting for robot to reach new calibration pose', 1);
        pause(2.0); % wait for stable position
        
        % get cam sample
        [T, timestamp, isVisible, message] = getLocatorTransformMatrix(camSocket, camInStream, camOutStream, camLocator);
        
        if (isVisible)
            camHTMs(:,:,acquisitionCounter) = T;
            disp(sprintf('\t%s', 'Got camera sample'));
            disp(camHTMs(:,:,acquisitionCounter));
            % get robot sample       
            robJoints(:,acquisitionCounter) = getJointPositions(robotSocket, robotInStream, robotOutStream );
            robHTMs(:,:,acquisitionCounter) = getHTM(robotSocket, robotInStream, robotOutStream );
            %robHTMs(:,:,acquisitionCounter) = directKinematics(robJoints(:,acquisitionCounter));
            disp(sprintf('\t%s', 'Got robot sample'));
            disp(robHTMs(:,:,acquisitionCounter));
            acquisitionCounter = acquisitionCounter + 1;
        else
            disp(sprintf('\t%s', 'Sample skipped: locator invisible'));
        end
    end

%% Save files
    save(robJointsFile, 'robJoints');
    save(robHTMsFile, 'robHTMs');
    save(camHTMsFile, 'camHTMs');