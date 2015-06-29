%% Ultrasound free hand calibration
initWS;

%% Measure the wire end-points using Cambar and a stylus
if (exist('data\zWireEndPoints.mw', 'file'))
    load('data\zWireEndPoints.mw', '-mat');
else
    [p, TAll] = zWirePhantomInCameraCoord('134.28.45.63');
end

% Four points for the Z-Phantom
p1Obs = [p(1,1:4:end); p(2,1:4:end); p(3,1:4:end)];
p2Obs = [p(1,2:4:end); p(2,2:4:end); p(3,2:4:end)];
p3Obs = [p(1,3:4:end); p(2,3:4:end); p(3,3:4:end)];
p4Obs = [p(1,4:4:end); p(2,4:4:end); p(3,4:4:end)];

meanPointsPhantom = [mean(p1Obs,2) mean(p2Obs,2) mean(p3Obs,2) mean(p4Obs,2)];

% Mean TF
meanTFs = [mean(TAll(:,1:4:end),2) mean(TAll(:,2:4:end),2) mean(TAll(:,3:4:end),2) mean(TAll(:,4:4:end),2)];
meanTF1 = reshape(meanTFs(:,1), 4,4);
meanTF2 = reshape(meanTFs(:,2), 4,4);
meanTF3 = reshape(meanTFs(:,3), 4,4);
meanTF4 = reshape(meanTFs(:,4), 4,4);

%% Checking if p1p2 and p3p4 are parallel
% If the z-wire doesnot have parallel lines then it will affect the
% accuracy of the calibration process. 
p1p2 = meanPointsPhantom(:,2)-meanPointsPhantom(:,1);
p2p3 = meanPointsPhantom(:,3)-meanPointsPhantom(:,2);
p3p4 = meanPointsPhantom(:,4)-meanPointsPhantom(:,3);
theta = acosd((p1p2./norm(p1p2))'*(p3p4./norm(p3p4)));

% Normal to the z-wire phantom - later used for visualization purposes.
normalToZWire = cross(p1p2, p2p3);
normalToZWire = normalToZWire./norm(normalToZWire);
%% Visualize Z-Wire phantom
figure;hold on;
plot3(meanPointsPhantom(1,:), meanPointsPhantom(2,:), meanPointsPhantom(3,:), '*-')
text(meanPointsPhantom(1,:), meanPointsPhantom(2,:), meanPointsPhantom(3,:), ['p1'; 'p2'; 'p3'; 'p4'])
rmsDistFromMean = [rmsDeviation(p1Obs, meanPointsPhantom(:,1)) rmsDeviation(p2Obs, meanPointsPhantom(:,2)) rmsDeviation(p3Obs, meanPointsPhantom(:,3)) rmsDeviation(p4Obs, meanPointsPhantom(:,4))];
[x,y,z] = sphere;
surf(rmsDistFromMean(1)*x+meanPointsPhantom(1,1),rmsDistFromMean(1)*y+meanPointsPhantom(2,1),rmsDistFromMean(1)*z+meanPointsPhantom(3,1),'EdgeColor','none')
surf(rmsDistFromMean(2)*x+meanPointsPhantom(1,2),rmsDistFromMean(2)*y+meanPointsPhantom(2,2),rmsDistFromMean(2)*z+meanPointsPhantom(3,2),'EdgeColor','none')
surf(rmsDistFromMean(3)*x+meanPointsPhantom(1,3),rmsDistFromMean(3)*y+meanPointsPhantom(2,3),rmsDistFromMean(3)*z+meanPointsPhantom(3,3),'EdgeColor','none')
surf(rmsDistFromMean(4)*x+meanPointsPhantom(1,4),rmsDistFromMean(4)*y+meanPointsPhantom(2,4),rmsDistFromMean(4)*z+meanPointsPhantom(3,4),'EdgeColor','none')

%% Acquire Ultrasound images and corresponding poses of the US probe
% Take at least 20 images. 
% Make sure that the depth level at which the z-wire phantom is located is
% in focus. You adjust this directly at the Ultrasonix device.
% Also, make sure contrast of the images is apropriate for good
% segmentation results.


%
%
%
%        CC          p2 ZZZZZZZZZZ p1
%    CCCCCC              ZZ              c3
%    Camera                 ZZZ          c2
%    CCCCCC                    ZZ        c1
%        CC          p4 ZZZZZZZZZZ p3
%
%
%

%% Segmenting Ultrasound images

% read probe poses
probePoses = importProbePoses('data\ultrasoundImagesAndPoses\probePoses.txt');

% take only the measurements with valid pose
imgMask = (1:20).*(probePoses(:,2)' == 1);
imgMask(imgMask==0) = [];
probePoses = probePoses(imgMask, 3:end);

% get number of valid images
numImages = size(probePoses,1);

% shift by -1 for getting filename suffices of the valid images
imgMask = imgMask-1;

% segement the points of the z-wire
[cc1, cc2, cc3, xmmPerPx, ymmPerPx, allImages] = segmentZPhantomPointsInUSImages('data\ultrasoundImagesAndPoses\fileout_', numImages,imgMask);

c1 = cc1;
c2 = cc2;
c3 = cc3;

% If the segmented locations are in pixels, convert them to 'mm'.
scaleMat = diag([xmmPerPx ymmPerPx]);
c1 = (scaleMat*reshape(c1',2,[]))';
c2 = (scaleMat*reshape(c2',2,[]))';
c3 = (scaleMat*reshape(c3',2,[]))';

% Calculate the distances c1c2, c2c3 and c1c3 in mm
% This is necessary for estimating the c2 (middle of the three cross-
% sectional points of the z-wire phantom. We can call this point 'zMid') in
% camera coordinate system.
d_c3c2 = zeros(numImages,1);
d_c1c3 = zeros(numImages,1);

for i = 1:numImages
    d_c3c2(i) = norm(c3(i,:)'-c2(i,:)');
    d_c1c3(i) = norm(c1(i,:)'-c3(i,:)');
end;
%% Find the global coordinates of middle of the three cross-sectional 
% points of the z-wire phanton in ultrasound images
d = d_c3c2 ./ d_c1c3;
zMidCamera = repmat(meanPointsPhantom(:,2)',numImages,1) + ...
    repmat(d,1,3) .* repmat(meanPointsPhantom(:,3)'-meanPointsPhantom(:,2)',numImages,1);

%% Estimating Tranformation between image coordinate system and US Probe coordinate system.

% While acquiring ultrasound images, you would have saved the corresponding
% pose of probe. All of these poses are saved in a text file. 

% Read probe poses
%probePoses = importProbePoses('data\ultrasoundImagesAndPoses\probePoses.txt');
% 4x4 matrix:
% timestamp, visible-flag, R00, R01, R02, X, R10, R11, R12, Y, R20, R21, R22, Z, 0, 0, 0, 1


% zMidCamera can be tranformed to Probe Coordinate System using the above
% poses
zMidProbe = zeros(numImages,3);
for i = 1:numImages
    tmp = inv(reshape(probePoses(i,:),4,4)') * [zMidCamera(i,:) 1]'; % reshape(..) gives transposed
    zMidProbe(i,:) = tmp(1:3)';
end;

% Appending 0 z-coordinate to the 2D segmented points in image coordinate
% system.
zMidImage = [c2 zeros(numImages,1)];

% Using least squares to estimate the transformation
[R,T,Yf,ErrUSP] = rot3dfit(zMidImage,zMidProbe);
tfMatImageToProbe = [[R' T']; [zeros(1,3) 1]];
% zInImageToCamera = ...
zInImageToCamera = zeros(numImages,3);
for i = 1:numImages
    tmp = reshape(probePoses(i,:),4,4)' * tfMatImageToProbe * [zMidImage(i,:) 1]';
    zInImageToCamera(i,:) = tmp(1:3)';
end;

%% Visualize the calibration error
% Comparing the zMidCamera and zInImageToCamera

%%%% Uncomment the following lines, once you have computed zMidCamera and 
%%%% zInImageToCamera

hold on; % plot over the last figure
zMidCompX = [zMidCamera(:,1)'; zInImageToCamera(:,1)'];
zMidCompY = [zMidCamera(:,2)'; zInImageToCamera(:,2)'];
zMidCompZ = [zMidCamera(:,3)'; zInImageToCamera(:,3)'];
plot3(zMidCamera(:,1), zMidCamera(:,2), zMidCamera(:,3), '*')
plot3(zMidCompX, zMidCompY, zMidCompZ, 'r+-')
hold off;
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')
title('Z-Wire Phantom and detected mid-points in camera reference frame')

calError = zeros(1,numImages);
for it = 1:numImages
    calError(it) = norm(zMidCamera(it,:)'-zInImageToCamera(it,:)');
end;
disp(['Mean calibration error in mm: ' num2str(mean(calError))]);

