function [X,Y,error] = heyecalib(poses_rob, poses_mark, num)

% load('poses_mark.mat');
% load('poses_rob.mat');
Ng = poses_mark;
M = poses_rob;

j = 0;
for i = 1:num
    if Ng{i}(1,1) ~= 0
        j = j+1;
        rotM = M{j}(1:3,1:3);
        N = inv(Ng{j}(:,:));
        transM = M{j}(1:3,4);
        
        rotA = [rotM*N(1,1),rotM*N(2,1),rotM*N(3,1),zeros(3);
            rotM*N(1,2),rotM*N(2,2),rotM*N(3,2),zeros(3);
            rotM*N(1,3),rotM*N(2,3),rotM*N(3,3),zeros(3);
            rotM*N(1,4),rotM*N(2,4),rotM*N(3,4),rotM];
        
        Ai = [rotA, -eye(12)];
        
        bi = [zeros(9,1); -transM];
        
        switch j
            case 1
                A = Ai;
                b = bi;
            otherwise
                A = [A;Ai];
                b = [b;bi];
        end
    end
end

[Q,R] = qr(A);
% R = R(1:24,1:24);
% Q = Q(:,1:24);
% w = inv(R)*(Q'*b);
w = R\Q'*b;
X = [w(1:3),w(4:6),w(7:9),w(10:12);0,0,0,1];
Y = [w(13:15),w(16:18),w(19:21),w(22:24);0,0,0,1];

[u,s,v]=svd(Y(1:3,1:3));
YR_orth = u*v';
Y_old = Y;
X_old = X;
Y(1:3,1:3) = YR_orth;
X = inv(poses_rob{1})*Y*poses_mark{1};

% Error

for i = 1:j
    error = sum(norm((M{i}(:,:)*X-Y*Ng{i}(:,:)), 'fro'));
end

end