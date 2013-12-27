%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Implement the optical flow approach from lecture to estimate
% the translation at every point. Use convolution to compute sums over W
% efficiently.

image1 = imread('../hw4_supp/images/hotel.seq00.png');
image2 = imread('../hw4_supp/images/hotel.seq05.png');

keypoints_x1 = 16;
keypoints_y1 = 16;
for indx = 16:32:625
    for indy = 16:32:465
        keypoints_x1 = [keypoints_x1, indx];
        keypoints_y1 = [keypoints_y1, indy];
    end
end

[keypoints_x2, keypoints_y2] = predictTranslationAll(keypoints_x1, keypoints_y1, image1, image2);