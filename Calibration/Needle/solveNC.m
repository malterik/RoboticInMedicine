function [ pCal, pPivot ] = solveNC( poses )
%solveNC Needle tip calibration
%
%   Performs needle tip calibration using hot spot/pivot calibration procedure. 
%
% parameters:
% poses, 4x4xn matrix, where n denotes the number of samples.

    % get number of samples
    n = size(poses,3);
    
    % create global matrix A and vector b
    A = zeros(3*n,6);
    b = zeros(2*n,1);
    
    % fill equations
    for i = 1:n;
        
        % create Ai and bi from sample
        Ai = zeros(3,6);
        bi = zeros(2,1);
        
        RPose = poses(1:3,1:3,i);
        TPose = poses(1:3,4,i);
        
        Ai = [RPose -eye(3)];
        bi = -TPose;
        
        % add to global A and b
        A(1+(i-1)*3:3+(i-1)*3,:) = Ai;
        b(1+(i-1)*3:3+(i-1)*3,1) = bi;
    end
    
    % solve equations
    x = linsolve(A,b);
    
    % set output
    pCal = x(1:3);
    pPivot = x(4:6);
end

