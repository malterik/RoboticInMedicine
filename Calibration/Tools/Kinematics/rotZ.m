function [ rotation ] = rotZ( angle )
%ROTZ Summary of this function goes here
%   Detailed explanation goes here

    rotation = [cosd(angle) -sind(angle) 0; sind(angle) cosd(angle) 0; 0 0 1];

end

