%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: This file implement the coarse-to-fine tracking procedure.
% Totest the accuracy on large translations, tracks the keypoints by only
% using frames F=[0,10,20,30,40,50]. Compare this to if you run original
% tracker over frames F. u in I, to estimate v in J.
% input:
% output:

image1 = imread('../hw4_supp/images/hotel.seq00.png');
image2 = imread('../hw4_supp/images/hotel.seq05.png');
load('../hw4_supp/tracked_points.mat');
u = [Xs(1, 1), Ys(1, 1)]';
v_gt = [Xs(5, 1), Ys(5, 1)]';
subsampling = 2;
nlevels = 4;
winsize = 15;

%function CoarseToFineTracking
%end

pyramid_image1 = KLTCreatePyramid(image1, subsampling, nlevels);
pyramid_image2 = KLTCreatePyramid(image2, subsampling, nlevels);
pyramid_guess = [0, 0]';
for ind = nlevels:-1:1
    u_l = u / (subsampling^(ind-1));
    gx = [-1, 0, 1];
    gy = [-1, 0, 1]';
    Ix = filter2(gx, pyramid_image1{ind}, 'same');
    Iy = filter2(gy, pyramid_image1{ind}, 'same');
    gxx = 0; gxy = 0; gyy = 0;
    
    for indx = -floor(winsize/2):floor(winsize/2)
        for indy = -floor(winsize/2):floor(winsize/2)
            Ix_interpolate = interpolate(u_l(1)+indx, u_l(2)+indy, Ix);
            Iy_interpolate = interpolate(u_l(1)+indx, u_l(2)+indy, Iy);
            gxx = gxx + Ix_interpolate*Ix_interpolate;
            gxy = gxy + Ix_interpolate*Iy_interpolate;
            gyy = gyy + Iy_interpolate*Iy_interpolate;
        end
    end
    G = [gxx, gxy; gxy, gyy];
    
    nu = [0, 0]';
    b = [0, 0]';
    iteration = 1;
    delta_It = zeros(size(pyramid_image1{ind}, 1), size(pyramid_image1{ind}, 2));
    for index_x = 1:size(pyramid_image1{ind}, 2)
        for index_y = 1:size(pyramid_image1{ind}, 1)
            J_interpolate = interpolate(index_x+pyramid_guess(1)+nu(1), index_y+pyramid_guess(2)+nu(2),...
                pyramid_image2{ind});
            delta_It(index_y, index_x) = pyramid_image1{ind}(index_y, index_x)...
                - J_interpolate;
        end
    end
    
    while 1
        for indx = -floor(winsize/2):floor(winsize/2)
            for indy = -floor(winsize/2):floor(winsize/2)
                Ix_interpolate = interpolate(u_l(1)+indx, u_l(2)+indy, Ix);
                Iy_interpolate = interpolate(u_l(1)+indx, u_l(2)+indy, Iy);
                delta_It_interpolate = interpolate(u_l(1)+indx, u_l(2)+indy, delta_It);
                b(1) = b(1) + Ix_interpolate*delta_It_interpolate;
                b(2) = b(2) + Iy_interpolate*delta_It_interpolate;
            end
        end
        eta = pinv(G) * b;
        nu = nu + eta;
        iteration = iteration + 1
        if  iteration > 20 || (eta(1) < 0.1 && eta(2) < 0.1) break; end
    end
    
    pyramid_guess = 2 * (pyramid_guess + nu);
end

v = u + pyramid_guess + nu;