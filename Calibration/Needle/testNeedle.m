clear all;
close all;

load('stylusEndEffector.mat');
load('stylusNeedleTip.mat');
load('handEyeData.mat');
rob2needle = csvread('rob2needle.csv');

stylusEndEffector_cam = stylusEndEffector;
stylusNeedleTip_cam = stylusNeedleTip;

% dist = norm(stylusNeedleTip(1:3,4)-stylusEndEffector(1:3,4))


stylusEndEffector_rob = Y*stylusEndEffector_cam;
[U,S,V] = svd(stylusEndEffector_rob(1:3,1:3));
stylusEndEffector_rob = [U*V' stylusEndEffector_rob(1:3,4); 0 0 0 1];

stylusNeedleTip_rob = Y*stylusNeedleTip_cam;
[U,S,V] = svd(stylusNeedleTip_rob(1:3,1:3));
stylusNeedleTip_rob = [U*V' stylusNeedleTip_rob(1:3,4); 0 0 0 1];
%stylusNeedleTip_rob = [stylusEndEffector_rob(1:3,1:3) stylusNeedleTip_rob(1:3,4); 0 0 0 1];
% stylusNeedleTip_rob = stylusEndEffector_rob*rob2needle;
% 
% zAxis = stylusNeedleTip_rob(1:3,4) - stylusEndEffector_rob(1:3,4);
% axisAngle = vrrotvec([0 0 1], zAxis);
% stylusNeedleTip_rob = [vrrotvec2mat(axisAngle) stylusNeedleTip_rob(1:3,4); 0 0 0 1]; 

% stylusEndEffector_rob
% stylusNeedleTip_rob
% 
% dist = norm(stylusNeedleTip_rob(1:3,4)-stylusEndEffector_rob(1:3,4))

robPose = stylusNeedleTip_rob*inv(rob2needle);

stretch = 1;

pr0 = [0 0 0 1]';
xr0 = stretch * [1 0 0 0]';
yr0 = stretch * [0 1 0 0]';
zr0 = stretch * [0 0 1 0]';

quiv_ee_cam = zeros(4,4);
quiv_nt_cam = zeros(4,4);
quiv_ee_rob = zeros(4,4);
quiv_nt_rob = zeros(4,4);
quiv_newpose_rob = zeros(4,4);

quiv_ee_cam(1,:) = stylusEndEffector_cam*pr0;
quiv_ee_cam(2,:) = stylusEndEffector_cam*xr0;
quiv_ee_cam(3,:) = stylusEndEffector_cam*yr0;
quiv_ee_cam(4,:) = stylusEndEffector_cam*zr0;

quiv_nt_cam(1,:) = stylusNeedleTip_cam*pr0;
quiv_nt_cam(2,:) = stylusNeedleTip_cam*xr0;
quiv_nt_cam(3,:) = stylusNeedleTip_cam*yr0;
quiv_nt_cam(4,:) = stylusNeedleTip_cam*zr0;

quiv_ee_rob(1,:) = stylusEndEffector_rob*pr0;
quiv_ee_rob(2,:) = stylusEndEffector_rob*xr0;
quiv_ee_rob(3,:) = stylusEndEffector_rob*yr0;
quiv_ee_rob(4,:) = stylusEndEffector_rob*zr0;

quiv_nt_rob(1,:) = stylusNeedleTip_rob*pr0;
quiv_nt_rob(2,:) = stylusNeedleTip_rob*xr0;
quiv_nt_rob(3,:) = stylusNeedleTip_rob*yr0;
quiv_nt_rob(4,:) = stylusNeedleTip_rob*zr0;

quiv_newpose_rob(1,:) = robPose*pr0;
quiv_newpose_rob(2,:) = robPose*xr0;
quiv_newpose_rob(3,:) = robPose*yr0;
quiv_newpose_rob(4,:) = robPose*zr0;

figure();
hold on
grid on;
quiver3(quiv_ee_cam(1,1),quiv_ee_cam(1,2),quiv_ee_cam(1,3),quiv_ee_cam(2,1),quiv_ee_cam(2,2),quiv_ee_cam(2,3),'r');
quiver3(quiv_ee_cam(1,1),quiv_ee_cam(1,2),quiv_ee_cam(1,3),quiv_ee_cam(3,1),quiv_ee_cam(3,2),quiv_ee_cam(3,3),'g');
quiver3(quiv_ee_cam(1,1),quiv_ee_cam(1,2),quiv_ee_cam(1,3),quiv_ee_cam(4,1),quiv_ee_cam(4,2),quiv_ee_cam(4,3),'b');

quiver3(quiv_nt_cam(1,1),quiv_nt_cam(1,2),quiv_nt_cam(1,3),quiv_nt_cam(2,1),quiv_nt_cam(2,2),quiv_nt_cam(2,3),'r');
quiver3(quiv_nt_cam(1,1),quiv_nt_cam(1,2),quiv_nt_cam(1,3),quiv_nt_cam(3,1),quiv_nt_cam(3,2),quiv_nt_cam(3,3),'g');
quiver3(quiv_nt_cam(1,1),quiv_nt_cam(1,2),quiv_nt_cam(1,3),quiv_nt_cam(4,1),quiv_nt_cam(4,2),quiv_nt_cam(4,3),'b');
hold off;

figure();
hold on
grid on;
quiver3(quiv_ee_rob(1,1),quiv_ee_rob(1,2),quiv_ee_rob(1,3),quiv_ee_rob(2,1),quiv_ee_rob(2,2),quiv_ee_rob(2,3),'r');
quiver3(quiv_ee_rob(1,1),quiv_ee_rob(1,2),quiv_ee_rob(1,3),quiv_ee_rob(3,1),quiv_ee_rob(3,2),quiv_ee_rob(3,3),'g');
quiver3(quiv_ee_rob(1,1),quiv_ee_rob(1,2),quiv_ee_rob(1,3),quiv_ee_rob(4,1),quiv_ee_rob(4,2),quiv_ee_rob(4,3),'b');

quiver3(quiv_nt_rob(1,1),quiv_nt_rob(1,2),quiv_nt_rob(1,3),quiv_nt_rob(2,1),quiv_nt_rob(2,2),quiv_nt_rob(2,3),'r');
quiver3(quiv_nt_rob(1,1),quiv_nt_rob(1,2),quiv_nt_rob(1,3),quiv_nt_rob(3,1),quiv_nt_rob(3,2),quiv_nt_rob(3,3),'g');
quiver3(quiv_nt_rob(1,1),quiv_nt_rob(1,2),quiv_nt_rob(1,3),quiv_nt_rob(4,1),quiv_nt_rob(4,2),quiv_nt_rob(4,3),'b');

quiver3(quiv_newpose_rob(1,1),quiv_newpose_rob(1,2),quiv_newpose_rob(1,3),quiv_newpose_rob(2,1),quiv_newpose_rob(2,2),quiv_newpose_rob(2,3),'k');
quiver3(quiv_newpose_rob(1,1),quiv_newpose_rob(1,2),quiv_newpose_rob(1,3),quiv_newpose_rob(3,1),quiv_newpose_rob(3,2),quiv_newpose_rob(3,3),'k');
quiver3(quiv_newpose_rob(1,1),quiv_newpose_rob(1,2),quiv_newpose_rob(1,3),quiv_newpose_rob(4,1),quiv_newpose_rob(4,2),quiv_newpose_rob(4,3),'k');
hold off;

