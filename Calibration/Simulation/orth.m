function [ pose ] = orth( HTM )
%ORTH Summary of this function goes here
%   Detailed explanation goes here
    [U,S,V] = svd(HTM(1:3,1:3));
    pose = [U*V' HTM(1:3,4); 0 0 0 1];
end

