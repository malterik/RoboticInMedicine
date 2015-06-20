function [ X,Y ] = handEyeErnst( M,N )
%HANDEYEERNST Summary of this function goes here
%   Performs hand eye calibration as proposed by Ernst et al.
%
% parameters:
% M, 4x4xL matrix, where L denotes the number of mesurements.
% N, 4x4xL matrix, where L denotes the number of mesurements.
    
    n = size(M,3);
    
    % create matrix A and vector b
    A = zeros(12*n,24); % a huge matrix for all the equations
    b = zeros(12*n,1); % solution vector for all the equations
    for i=1:n

        Ni = inv(N(:,:,i));
        Mi = M(:,:,i);
        
        % rotational part of the robot matrix
        RMi = (Mi(1:3,1:3));
        
        % Create matrix A for sample i
        Ai=zeros(12,24);
            
            % left half
            Ai(1:3,1:3)=RMi*Ni(1,1);
            Ai(4:6,1:3)=RMi*Ni(1,2);
            Ai(7:9,1:3)=RMi*Ni(1,3);
            Ai(10:12,1:3)=RMi*Ni(1,4);

            Ai(1:3,4:6)=RMi*Ni(2,1);
            Ai(4:6,4:6)=RMi*Ni(2,2);
            Ai(7:9,4:6)=RMi*Ni(2,3);
            Ai(10:12,4:6)=RMi*Ni(2,4);

            Ai(1:3,7:9)=RMi*Ni(3,1);
            Ai(4:6,7:9)=RMi*Ni(3,2);
            Ai(7:9,7:9)=RMi*Ni(3,3);
            Ai(10:12,7:9)=RMi*Ni(3,4);
            
            % right half    
            Ai(1:9,10:12)=0;
            Ai(10:12,10:12)=RMi;
            Ai(1:12,13:24)= -eye(12);
        
        % create solution vector for sample i
        bi=zeros(12,1);
        bi(10:12,1)= -Mi(1:3,4);
        
        % store in global matrix A and solution vector b
        A(1+(i-1)*12:12+(i-1)*12,:) = Ai;
        b(1+(i-1)*12:12+(i-1)*12,1) = bi;       
    end
    
    % solve using QR-Factorization
    [Q, R] = qr(A);
    Z = Q'*b;
    W = R\Z;


    % get X and Y from W
    X = [reshape(W(1:12), [3 4]); 0 0 0 1];
    Y = [reshape(W(13:24), [3 4]); 0 0 0 1];
end

