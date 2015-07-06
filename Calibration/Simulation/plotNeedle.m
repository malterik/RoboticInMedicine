function [] = plotNeedle( robPose, rob2needle, figHandle )
%PLOTHTM Summary of this function goes here
%   Detailed explanation goes here 
    needlePose = robPose * rob2needle;
    needleDir = needlePose*[0 0 1 0]';
    needleDir = needleDir(1:3);
    needleDir = needleDir/norm(needleDir);
    needleStart = needlePose(1:3,4) - (needleDir)*(0.21);
    direction =  needlePose(1:3,4) - needleStart;
    
    figure(figHandle);
    hold on;
    quiver3(needleStart(1), needleStart(2), needleStart(3), direction(1), direction(2), direction(3), 0, 'k');
    hold off;
end

