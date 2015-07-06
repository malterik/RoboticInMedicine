% Handles the data aqcuisition for the hand eye calibration.
% A given number of calibration frames is acquired at poses within a
% maximum translation and a maximum rotation around a random axis.

%% Definitions
    % Network settrings

    robotIP = '134.28.45.95'; % robot ip
    robotPort = 5005; % robot port    
    timeout = 3000; % timeout for tcp reads
        
    % Output settings
    startPoseFile = 'Output/startPose.mat'; % output file for robot joints

%% Initialization  
    % Connect to RobServer, if necessary
    if (~(exist('robotSocket', 'var')))
        [robotSocket, robotInStream, robotOutStream] = initRobot(robotIP, robotPort, timeout);
    end;

%% Get joint positions from robot
    startJoints = getJointPositions(robotSocket, robotInStream, robotOutStream );
    disp(startJoints);
    %startPose = directKinematics(startJoints);
    %startPose = startPose.startPose;
    startPose = getHTM(robotSocket, robotInStream, robotOutStream );
    startPose(1:3,4) = startPose(1:3,4)/1000;
    disp(startPose);

%% Save joints to file
    save(startPoseFile, 'startPose');