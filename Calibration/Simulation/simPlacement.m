% clear all;
% close all;

%% Initialization
    % load Files
    load('Calibration/Data/stylusEndEffector.mat');
    load('Calibration/Data/stylusNeedleTip.mat');
    load('Calibration/Data/handEyeData.mat');
    rob2needle = csvread('Input/rob2needle.csv');
    %windowPoints = csvread('Input/windowPoints.csv');
    windowPoints = csvread('Input/windowPoints_backup.csv'); 
    
    % create figure;
    simFig = figure(1);
    hold on;
    grid on;
    hold off;
    
            
    xStart = [0.4 0 0.2];
    yStart = [0 0.9 0.2]
    diff = (yStart-xStart);
    
%     figure(simFig);
%     hold on;
%     plot3([xStart(1) yStart(1)], [xStart(2) yStart(2)], [xStart(3) yStart(3)], 'k-');
%     hold off;
%     

%% Create new coordinates
    windowPointTranslations_rob(1,:) = xStart+diff*0.4;
    windowPointTranslations_rob(2,:) = xStart+diff*0.6;
    windowPointTranslations_rob(3,:) = windowPointTranslations_rob(2,:) + [0 0 0.1];
    windowPointTranslations_rob(4,:) = windowPointTranslations_rob(1,:) + [0 0 0.1];
    
    windowPointsHTM_cam = zeros(4,4,size(windowPointTranslations_rob,1));
    windowPoints= zeros(size(windowPointTranslations_rob,1), 3);
    for i = 1:size(windowPoints,1);
        windowPointsHTM_cam(:,:,i) = [eye(3) windowPointTranslations_rob(i,:)'; 0 0 0 1];
        windowPointsHTM_cam(:,:,i) = inv(Y)*windowPointsHTM_cam(:,:,i);
        windowPointsHTM_cam(:,:,i) = orth(windowPointsHTM_cam(:,:,i));
        windowPoints(i,:) = windowPointsHTM_cam(1:3,4,i);
    end
    
    
    
%% Create tumor
    
    % create tumor on line perpendicular to plane in some distance
    distance = 0.1; % distance from window
    data = [windowPoints(:,1) windowPoints(:,2) windowPoints(:,3)];
    C = fitNormal(data);
    C_norm = C/norm(C);
    windowMiddle = mean(windowPoints);
    tumorPoint = windowMiddle' - distance*C_norm;
    
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
    distOutside = 0.05;
    data = [windowPointTranslations_rob(:,1) windowPointTranslations_rob(:,2) windowPointTranslations_rob(:,3)];
    C_rob = fitNormal(data);
    C_norm_rob = C_rob/norm(C_rob);
    outSidePoint = windowMiddle_rob' + distOutside*C_norm_rob;
    
    % add to plot
%     figure(simFig);  
%     hold on;
%     plot3(outSidePoint(1), outSidePoint(2), outSidePoint(3), 'rx', 'MarkerSize', 25);
%     alpha(0.1);
%     hold off;
    
%% Get points from robot movement
    
%     outside_z_offset_pose = csvread('Output/sim_outside_z_offset_matrix.csv');
%     plotHTM(outside_z_offset_pose, simFig, 0.04);
%     plotNeedle(outside_z_offset_pose, rob2needle, simFig);
%     
    outside_pose = csvread('Output/sim_outside_matrix.csv');
    plotHTM(outside_pose, simFig, 0.04);
    plotNeedle(outside_pose, rob2needle, simFig);
% 
% %     outside_right_orientaion_pose = csvread('Output/sim_outside_right_orientation_matrix.csv');
% %     plotHTM(outside_right_orientaion_pose, simFig, 0.04);
% %     plotNeedle(outside_right_orientaion_pose, rob2needle, simFig);
% 
    final_pose_pre = csvread('Output/sim_final_matrix_pre.csv');
    plotHTM(final_pose_pre, simFig, 0.04);
    plotNeedle(final_pose_pre, rob2needle, simFig);

    final_pose = csvread('Output/sim_final_matrix.csv');
    plotHTM(final_pose, simFig, 0.04);
    plotNeedle(final_pose, rob2needle, simFig);
    
%     f_pose = [0.104574 -0.056659 -0.992898 -0.544405; -0.799387 0.589151 -0.117811 -0.133997; 0.591650 0.806031 0.016318 0.166563; 0 0 0 1];
%     plotNeedle(f_pose, rob2needle, simFig);



    csvwrite('Input/windowPoints.csv', windowPoints);
    csvwrite('Input/tumorCenter.csv', tumorPoint);
