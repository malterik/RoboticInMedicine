clear all;
close all;

%% Initialization
    % load Files
    load('Calibration/Data/stylusEndEffector.mat');
    load('Calibration/Data/stylusNeedleTip.mat');
    load('Calibration/Data/handEyeData.mat');
    rob2needle = csvread('Input/rob2needle.csv');
    windowPoints = csvread('Input/windowPoints.csv');
    tumorPoint = csvread('Input/tumorCenter.csv');
    load('Output/startPose.mat');
    
    % create figure;
    simFig = figure(1);
    hold on;
    grid on;
    hold off;

%% Transform window to rob world
    % Transform window coordinates to rob world
    windowPointsHTM_rob = zeros(4,4,size(windowPoints,1));
    windowPointTranslations_rob = zeros(size(windowPoints,1), 3);
    for i = 1:size(windowPoints,1);
        windowPointsHTM_rob(:,:,i) =  [eye(3) windowPoints(i,:)'; 0 0 0 1];
        windowPointsHTM_rob(:,:,i) = convertCamToRob(Y,windowPointsHTM_rob(:,:,i));
        windowPointTranslations_rob(i,:) = windowPointsHTM_rob(1:3,4,i);
    end
    
    % Calculate middle of window in rob world
    windowMiddle_rob = mean(windowPointTranslations_rob);
    
    windowLachs(1,:) = (windowMiddle_rob+windowPointTranslations_rob(1,:))/2;
    windowLachs(2,:) = (windowMiddle_rob+windowPointTranslations_rob(2,:))/2;
    windowLachs(3,:) = (windowMiddle_rob+windowPointTranslations_rob(3,:))/2;
    windowLachs(4,:) = (windowMiddle_rob+windowPointTranslations_rob(4,:))/2;
    windowLachs(5,:) = (windowLachs(1,:)+windowLachs(2,:))/2;
    windowLachs(6,:) = (windowLachs(2,:)+windowLachs(3,:))/2;
    windowLachs(7,:) = (windowLachs(3,:)+windowLachs(4,:))/2;
    windowLachs(8,:) = (windowLachs(4,:)+windowLachs(1,:))/2;
    
    % add window to plot
    figure(simFig);
    hold on;    
    plot3(windowPointTranslations_rob(:,1), windowPointTranslations_rob(:,2), windowPointTranslations_rob(:,3), 'ro');
    plot3(windowLachs(:,1), windowLachs(:,2), windowLachs(:,3), 'ko');
    plot3(windowMiddle_rob(1), windowMiddle_rob(2), windowMiddle_rob(3), 'rx', 'MarkerSize', 25);
    fill3(windowPointTranslations_rob(:,1), windowPointTranslations_rob(:,2), windowPointTranslations_rob(:,3), 'r');
    alpha(0.1);
    hold off;
    
%% Transform tumor to rob world
    
    % create tumor on line perpendicular to plane in some distance
    tumorPoint_rob = convertCamToRob(Y,[eye(3) tumorPoint; 0 0 0 1]);
    tumorPoint_rob = tumorPoint_rob(1:3,4);
    r = 0.01; % tumor radius

    
    % create sphere in tumor point
    [x,y,z] = sphere;
    x = r*x + tumorPoint_rob(1);
    y = r*y + tumorPoint_rob(2);
    z = r*z + tumorPoint_rob(3);  
    
    % add tumor to plot
    figure(simFig);  
    hold on;
    plot3(tumorPoint_rob(1), tumorPoint_rob(2), tumorPoint_rob(3), 'rx', 'MarkerSize', 25);
    hSurface = surf(x,y,z);
    set(hSurface,'FaceColor',[1 0 0],'FaceAlpha',0.5,'EdgeColor','none');
    alpha(0.1);
    hold off;
    
    
%% Get points from robot movement     
    % plot start pose
    plotHTM(startPose, simFig, 0.04);
    plotNeedle(startPose, rob2needle, simFig);

    % pose outside of box
    outside_pose = csvread('Output/sim_outside_matrix.csv');
    plotHTM(outside_pose, simFig, 0.04);
    plotNeedle(outside_pose, rob2needle, simFig);

    % poses from linear movement
    filecount = 0;
    while(1)
        fileName = sprintf('Output/sim_final_matrix%i.csv', filecount);
        try
            final_pose = csvread(fileName);
            filecount = filecount + 1;
            plotHTM(final_pose, simFig, 0.04);
            plotNeedle(final_pose, rob2needle, simFig);
        catch ME
            break;
        end
    end