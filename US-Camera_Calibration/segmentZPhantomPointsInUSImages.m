function [c1, c2, c3, xmmPerPx, ymmPerPx, allImages, invalidImgs] = segmentZPhantomPointsInUSImages(filenamePref, numImages, imgMask)
%SEGMENTZPHANTOMPOINTSINUSIMAGES Segments the Ultrasound images
%   SEGMENTZPHANTOMPOINTSINUSIMAGES(FILENAMEPREF, NUMIMAGES)
%   Reads in NUMIMAGES images with the prefix FILENAMEPREF and outputs the
%   locations of the cross-sectional points of z-wire phantom.
%
%   Modified 22th May 2015
%   Omer Rajput - MTEC, TUHH.

% Read images
I = imread([filenamePref '0.jpg']);

% These are just example of expected segmentation results. You would
% eventually not need the following two lines.
%filenamePrefSplit = strsplit(filenamePref, {'/','\'});

%filenameSegPref = [strjoin(filenamePrefSplit(1:end-1),'\') '\seg\seg'];
%ISeg = imread([filenameSegPref '0.jpg']);

% creating a figure with dummy structure
%scrsz = get(groot,'ScreenSize');
%figure('Position',[10 scrsz(4)*3/4 scrsz(3)*3/4 scrsz(4)*3/4]);

%subplot(1,2,1)
%ims = imshow(I);
%title('Image from Ultrasound device')

%subplot(1,2,2)
%ims1 = imshow(ISeg);
%title('Segmented z-wire cross-sectional points')

sz = size(I);
allImages = uint8(zeros(sz(1),sz(2),numImages));

c1 = zeros(numImages,2);
c2 = zeros(numImages,2);
c3 = zeros(numImages,2);

xmmPerPxAll = zeros(1,numImages);
ymmPerPx = 40.0 / 480; % known

fhResult = figure;
invalidImgs = false(numImages,1);

for idx = 0:numImages-1
    disp(['Segmentation of image.. ' num2str(imgMask(idx+1))]);
    % Reading the image
    allImages(:,:,idx+1) = imread([filenamePref num2str(imgMask(idx+1)) '.jpg']);
    I = allImages(:,:,idx+1);

    
    % define ROI containing the 3 points
    fhRoi = figure;
    imshow(imcomplement(imadjust(I)));
    mask = roipoly;
    close(fhRoi);
    I(~mask) = 0;
    % binarize
    th = graythresh(I);
    I = im2bw(I,th);
    % detect connected regions
    CC = bwconncomp(I,4);
    if CC.NumObjects ~= 3
        warning(['Detected ',num2str(CC.NumObjects),' objects! Discarding image.']);
        invalidImgs(idx+1) = true;
        continue;
    end;
    % compute center of the connected regions
    c = zeros(3,2);
    for i = 1:3
        [row,col] = ind2sub(sz,CC.PixelIdxList{i});
        c(i,:) = [mean(row), mean(col)];
    end;
    c1(idx+1,:) = c(1,:);
    c2(idx+1,:) = c(2,:);
    c3(idx+1,:) = c(3,:);
    
    
    figure(fhResult);
    imshow(allImages(:,:,idx+1));
    hold on;
    plot(c(1,2),c(1,1),'r+');
    plot(c(2,2),c(2,1),'g+');
    plot(c(3,2),c(3,1),'c+');
    
    % It is also helpful to estimate mm per pixel
    % Counting the white pixels in each column, to find out the width in
    % pixels. This will correspond to 38mm (from the probe).
    sumRow = sum(allImages(:,:,idx+1),1);
    indF = find(sumRow, 1, 'first');
    indL = find(sumRow, 1, 'last');
    widthPxUS = indL - indF + 1;
    xmmPerPxAll(idx+1) = 38.0 / widthPxUS;
    pause(1);
end

% discard invalid images
c1(invalidImgs,:) = [];
c2(invalidImgs,:) = [];
c3(invalidImgs,:) = [];
allImages(:,:,invalidImgs) = [];
xmmPerPxAll(invalidImgs) = [];


% mean of xmmPerPx
xmmPerPx = mean(xmmPerPxAll);

close(fhResult);