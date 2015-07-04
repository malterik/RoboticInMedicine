function [ pose ] = convertCamToRob( Y, HTM )
%CONVERTCAMTOROB Summary of this function goes here
%   Detailed explanation goes here
    pose = orth(Y*HTM);
end

