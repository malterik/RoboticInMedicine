function [ rotation ] = rotX( angle )
%ROTX Summary of this function goes here
%   Detailed explanation goes here

    rotation = [cosd(angle) 0 sind(angle); 0 1 0; -sind(angle) 0 cosd(angle)];
end

