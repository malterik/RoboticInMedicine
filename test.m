load('Output/startPose.mat');
load('Calibration\Data\usTest.mat');

a = Y*camHTM
b = startPose*rob2needle
norm(a(1:3,4)-b(1:3,4))