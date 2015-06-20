function [ rotation ] = rotX( angle )
%ROTX Summary of this function goes here
%   Detailed explanation goes here

    rotation = [1 0 0; 0 cosd(angle) -sind(angle); 0 sind(angle) cosd(angle)];
end

