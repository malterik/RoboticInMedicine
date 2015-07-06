function [] = plotHTM( HTM, figHandle, stretch )
%PLOTHTM Summary of this function goes here
%   Detailed explanation goes here

    pr0 = [0 0 0 1]';
    xr0 = stretch * [1 0 0 0]';
    yr0 = stretch * [0 1 0 0]';
    zr0 = stretch * [0 0 1 0]';

    quivHTM = zeros(4,4);

    quivHTM(1,:) = HTM*pr0;
    quivHTM(2,:) = HTM*xr0;
    quivHTM(3,:) = HTM*yr0;
    quivHTM(4,:) = HTM*zr0;
    
    figure(figHandle);
    hold on;
    quiver3(quivHTM(1,1),quivHTM(1,2),quivHTM(1,3),quivHTM(2,1),quivHTM(2,2),quivHTM(2,3),'r');
    quiver3(quivHTM(1,1),quivHTM(1,2),quivHTM(1,3),quivHTM(3,1),quivHTM(3,2),quivHTM(3,3),'g');
    quiver3(quivHTM(1,1),quivHTM(1,2),quivHTM(1,3),quivHTM(4,1),quivHTM(4,2),quivHTM(4,3),'b');
    hold off;
end

