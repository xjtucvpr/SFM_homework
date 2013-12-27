%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: referenced by Shape and Motion from Image Streams under
% Orthography.

%% abstract data
load('../hw4_supp/tracked_points.mat');
addpath('../hw4_supp/images');
ImgfileDir='D:\Matlab_workspace\Homework\hw4_supp\images\';  %color
vistail= '.png';
frames=dir([ImgfileDir  '*' vistail] );
nFrame=length(frames);
for ind = 1:nFrame
    figure(1); imshow(frames(ind).name); hold on;
    plot(Xs(ind,:), Ys(ind,:), 'g.', 'linewidth', 3.0);
    waitforbuttonpress;
end

%% The Factorization Method
U = Xs; % FxP
V = Ys; % FxP
W = [U; V]; % measurement matrix 2FxP
U_tilde = U - sum(U,2)/size(U,2)*ones(1,size(U,2)); % FxP
V_tilde = V - sum(V,2)/size(V,2)*ones(1,size(V,2)); % FxP
W_tilde = [U_tilde; V_tilde]; % registered measurement matrix 2FxP

[O1, Diagnoal, O2T] = svd(W_tilde); % compute the singular-value decomposition
O2 = O2T';
O1_part = O1(:, 1:3); % 2Fx3
Diagnoal_part = Diagnoal(1:3, 1:3); % 3x3
O2_part = O2(1:3, :); % 3xP

W_hat = O1_part * Diagnoal_part * O2_part; % best possible rank-3 approximation
Diagnoal_part_sqrt = sqrt(Diagnoal_part);
R_hat = O1_part * Diagnoal_part_sqrt; % the primes refer to the block partitioning
S_hat = Diagnoal_part_sqrt * O2_part;

Q = orthometric(R_hat); % Compute matrix Q by imposing the metric constraints

R = R_hat * Q; % Compute rotation matrix R
S = inv(Q) * S_hat; % Compute shape matrix S

i1 = R(1,:)';
i1 = i1 / norm(i1);
j1 = R(size(U,1)+1,:)';
j1 = j1 / norm(j1);
k1 = cross(i1, j1);
k1 = k1 / norm(k1);
R0 = [i1 j1 k1];
R = R * R0;
S = inv(R0) * S;

% reconstruct approximated W
t = [sum(U,2)/size(U,2) ;sum(V,2)/size(V,2)];
W = R*S + repmat(t, 1, size(U,2));
W = round(W);
I = imread('hotel.seq00.png');
figure;
imshow(I);
hold on;
for j=1:size(U,2)
    plot(W(1:size(U,1), j), W(size(U,1)+(1:size(U,1)), j), '-g');
end
plot(W(1, :), W(size(U,1)+1, :), '.y');

% % Motion
% % atan(y/x) yaw
% figure; plot(1:size(U,1), atan(R(1:size(U,1),2)./R(1:size(U,1),1)) * 180/pi);
% title('yaw'); xlabel('Frame number'); ylabel('degree');
% % atan(z/y) roll
% figure; plot(1:size(U,1), atan(R(1:size(U,1),3)./R(1:size(U,1),2)) * 180/pi);
% title('roll'); xlabel('Frame number'); ylabel('degree');
% % atan(z/x) pitch
% figure; plot(1:size(U,1), atan(R(1:size(U,1),3)./R(1:size(U,1),1)) * 180/pi);
% title('pitch'); xlabel('Frame number'); ylabel('degree');

% Shape, use rotate3 button
figure; plot3(S(1, :), S(2, :), S(3, :), '.');
figure; plot3(S(1, :), S(3, :), S(2, :), '.');
figure; plot3(S(2, :), S(1, :), S(3, :), '.');
figure; plot3(S(2, :), S(3, :), S(2, :), '.');
figure; plot3(S(3, :), S(1, :), S(2, :), '.');
figure; plot3(S(3, :), S(2, :), S(1, :), '.'); 
x = S(1, :);
y = S(2, :);
z = S(3, :);
tri = delaunay(x,y);
h = trisurf(tri, x, y, z);

k_f = zeros(3, size(U,1));
for index = 1 : size(U,1)
    i_f = R(index,:)';
    i_f = i_f / norm(i_f);
    j_f = R(size(U,1)+index,:)';
    j_f = j_f / norm(j_f);
    k = cross(i_f, j_f);
    k_f(:,index) = k / norm(k);
end
figure; plot3(k_f(1,:), k_f(2,:), k_f(3,:), 'y.');
figure; plot(1:size(U,1), k_f(1,:), 'r.');
figure; plot(1:size(U,1), k_f(2,:), 'g.');
figure; plot(1:size(U,1), k_f(3,:), 'b.');
