function [ X,Y ] = handEyeErnst( M,N )
%HANDEYEERNST Hand-eye calibration
%
%   Performs hand eye calibration as proposed in 
%   'Non-orthogonal tool/flange and robot/world calibration
%   by Ernst et al., Lübeck, 2012.
%
% parameters:
% M, 4x4xn matrix, where n denotes the number of samples.
% N, 4x4xn matrix, where n denotes the number of samples.
    
    % get number of samples
    n = size(M,3);
    
    % create global matrix A and vector b
    A = zeros(12*n,24);
    b = zeros(12*n,1);
    
    % fill equations
    for i=1:n     

        % use notation from paper
        Ai=zeros(12,24);
        NiInv = inv(N(:,:,i));
        Mi = M(:,:,i);
        RMi = Mi(1:3,1:3);
        TMi = Mi(1:3,4);

        % left half of Ai
        Ai(1:12,1:12) = [   RMi*NiInv(1,1) RMi*NiInv(2,1) RMi*NiInv(3,1) zeros(3);
                            RMi*NiInv(1,2) RMi*NiInv(2,2) RMi*NiInv(3,2) zeros(3);
                            RMi*NiInv(1,3) RMi*NiInv(2,3) RMi*NiInv(3,3) zeros(3);
                            RMi*NiInv(1,4) RMi*NiInv(2,4) RMi*NiInv(3,4) RMi];

        % right half of Ai
        Ai(1:12,13:24)= -eye(12);

        % local vector bi
        bi = zeros(12,1);
        bi(10:12,1) = -TMi;

        % add samples matrix to global matrix
        A(1+(i-1)*12:12+(i-1)*12,:) = Ai;
        b(1+(i-1)*12:12+(i-1)*12,1) = bi;
    end
    
    % solve equations
    w = linsolve(A,b);
    
%     % solve using QR-Factorization
%     [Q, R] = qr(A);
%     z = Q'*b;
%     w = R\z;
    
    % create matrices X and Y from solution vector w
    X = [reshape(w(1:12), [3 4]); 0 0 0 1];
    Y = [reshape(w(13:24), [3 4]); 0 0 0 1];
end

