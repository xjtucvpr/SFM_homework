%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Some keypoints will fall out of frame, or come into frame
% throughout the sequence. Use the matrix factorization to fill in the
% missing data and visualize the predicted positions of points that aren't
% visible in a particular frame. 
% Reference: Shape and Motion from Image Streams under Orthogography: a
% Factorization Method.

load('../hw4_supp/tracked_points.mat');
% function [occl_x, occl_y] = FillInMissingData()
    u11 = Xs(1,1); u12 = Xs(1,2); u13 = Xs(1,3); u14 = Xs(1,4);
    u21 = Xs(2,1); u22 = Xs(2,2); u23 = Xs(2,3); u24 = Xs(2,4);
    u31 = Xs(3,1); u32 = Xs(3,2); u33 = Xs(3,3); u34 = Xs(3,4);
    u41 = Xs(4,1); u42 = Xs(4,2); u43 = Xs(4,3); u44_gt = Xs(4,4);
    
    v11 = Ys(1,1); v12 = Ys(1,2); v13 = Ys(1,3); v14 = Ys(1,4);
    v21 = Ys(2,1); v22 = Ys(2,2); v23 = Ys(2,3); v24 = Ys(2,4);
    v31 = Ys(3,1); v32 = Ys(3,2); v33 = Ys(3,3); v34 = Ys(3,4);
    v41 = Ys(4,1); v42 = Ys(4,2); v43 = Ys(4,3); v44_gt = Ys(4,4);
    
    W_6x4 = [u11, u12, u13, u14;...
             u21, u22, u23, u24;...
             u31, u32, u33, u34;...
             v11, v12, v13, v14;...
             v21, v22, v23, v24;...
             v31, v32, v33, v34];
         
    [O1, Diagnoal, O2T] = svd(W_6x4); % compute the singular-value decomposition
    O2 = O2T';
    O1_part = O1(:, 1:3); % 6x3
    Diagnoal_part = Diagnoal(1:3, 1:3); % 3x3
    O2_part = O2(1:3, :); % 3x4

    W_hat = O1_part * Diagnoal_part * O2_part; % best possible rank-3 approximation
    Diagnoal_part_sqrt = sqrt(Diagnoal_part);
    R_hat = O1_part * Diagnoal_part_sqrt; % the primes refer to the block partitioning
    S_hat = Diagnoal_part_sqrt * O2_part;

    Q = orthometric(R_hat); % Compute matrix Q by imposing the metric constraints

    R_6x3 = R_hat * Q; % Compute rotation matrix R
    
%     i1 = R(1,:)';
%     i1 = i1 / norm(i1);
%     j1 = R(4,:)';
%     j1 = j1 / norm(j1);
%     k1 = cross(i1, j1);
%     k1 = k1 / norm(k1);
%     R0 = [i1 j1 k1];
%     R_6x3 = R_6x3 * R0;
    t_6x1 = [sum(W_6x4,2)/size(W_6x4,2)];
    
    S = pinv(R_6x3) * (W_6x4 - t_6x1*[1, 1, 1, 1]);
    
    c = 1/3 * (S(:,1)+S(:,2)+S(:,3));
    a4_referred = 1/3 * (u41 + u42 + u43);
    b4_referred = 1/3 * (v41 + v42 + v43);
    S1_referred = S(:,1) - c;
    S2_referred = S(:,2) - c;
    S3_referred = S(:,3) - c;
    
    u41_referred = u41 - a4_referred;
    u42_referred = u42 - a4_referred;
    u43_referred = u43 - a4_referred;
    v41_referred = v41 - b4_referred;
    v42_referred = v42 - b4_referred;
    v43_referred = v43 - b4_referred;
    
    i4_reffered = [u41_referred, u42_referred, u43_referred] ...
        * pinv([S1_referred, S2_referred, S3_referred]);
    j4_reffered = [v41_referred, v42_referred, v43_referred] ...
        * pinv([S1_referred, S2_referred, S3_referred]);
    R = [R_6x3(1,:);R_6x3(2,:);R_6x3(3,:);i4_reffered;...
        R_6x3(4,:);R_6x3(5,:);R_6x3(6,:);j4_reffered];
    W = [u11, u12, u13, u14;...
         u21, u22, u23, u24;...
         u31, u32, u33, u34;...
         u41, u42, u43, 1;...
         v11, v12, v13, v14;...
         v21, v22, v23, v24;...
         v31, v32, v33, v34;...
         v41, v42, v43, 1];
    t = 1/3 * (W - R*S) * [1; 1; 1; 0];
    s4 = S(:,4);
    u44 = i4_reffered * s4 + t(4);
    v44 = j4_reffered * s4 + t(8);
% end