%% Handles the data aqcuisition for the hand eye calibration.
% A given number of calibration frames is acquired at poses within a
% maximum translation and a maximum rotation around a random axis.

%% Definitions
    % Network settrings
    robotIP = '192.168.56.101'; % robot ip
    robotPort = 5005; % robot port    
    timeout = 3000; % timeout for tcp reads
        
    % Output settings
    startJointsFile = 'Data\robStartJoints.mat'; % output file for robot joints

%% Initialization  
    % Connect to RobServer, if necessary
    if (~(exist('robotSocket', 'var')))
        [robotSocket, robotInStream, robotOutStream] = initRobot(robotIP, robotPort, timeout);
    end;

%% Get joint positions from robot
    defaultJoints = getJointPositions(robotSocket, robotInStream, robotOutStream );
    disp(defaultJoints);

%% Save joints to file
    save(startJointsFile, 'defaultJoints');