clear all;
close all;

%% Initialization
    % load Files
    load('Calibration/Data/stylusEndEffector.mat');
    load('Calibration/Data/stylusNeedleTip.mat');
    load('Calibration/Data/handEyeData.mat');
    rob2needle = csvread('Input/rob2needle.csv');
    windowPoints = csvread('Input/windowPoints.csv');

    % create figure;
    simFig = figure();
    
%% Create tumor
    
    % create tumor on line perpendicular to plane in some distance
    distance = 0.1; % distance from window
    data = [windowPoints(:,1) windowPoints(:,2) windowPoints(:,3)];
    C = fitNormal(data);
    C_norm = C/norm(C);
    windowMiddle = mean(windowPoints);
    tumorPoint = windowMiddle' + distance*C_norm;
    
%% Create window
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
    
    % add window to plot
    figure(simFig);
    hold on;
    plot3(windowPointTranslations_rob(:,1), windowPointTranslations_rob(:,2), windowPointTranslations_rob(:,3), 'ro');
    plot3(windowMiddle_rob(1), windowMiddle_rob(2), windowMiddle_rob(3), 'rx', 'MarkerSize', 25);
    fill3(windowPointTranslations_rob(:,1), windowPointTranslations_rob(:,2), windowPointTranslations_rob(:,3), 'r');
    alpha(0.1);
    hold off;
    
%% Create tumor
    
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
    
%% Calculate position outside
    distOutside = 0.3;
    data = [windowPointTranslations_rob(:,1) windowPointTranslations_rob(:,2) windowPointTranslations_rob(:,3)];
    C_rob = fitNormal(data);
    C_norm_rob = C_rob/norm(C_rob);
    outSidePoint = windowMiddle_rob' + distOutside*C_norm_rob;
    
    % add to plot
    figure(simFig);  
    hold on;
    plot3(outSidePoint(1), outSidePoint(2), outSidePoint(3), 'rx', 'MarkerSize', 25);
    alpha(0.1);
    hold off;
    
%% Get points from robot movement
    outside_z_offset_pose = csvread('Output/sim_outside_z_offset_matrix.csv');
    plotHTM(outside_z_offset_pose,simFig,0.05);

    outside_pose = csvread('Output/sim_outside_matrix.csv');
    plotHTM(outside_pose,simFig,0.05);
    
    outside_right_orientaion_pose = csvread('Output/sim_outside_right_orientation_matrix.csv');
    plotHTM(outside_right_orientaion_pose,simFig,0.05);
    
    final_pose = csvread('Output/sim_final_matrix.csv');
    plotHTM(final_pose,simFig,0.05);
