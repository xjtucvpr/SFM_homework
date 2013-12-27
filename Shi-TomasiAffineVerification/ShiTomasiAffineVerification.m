%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Shi-Tomasi Affine Verification, referenced by Good features
% to track.
% input: image1, image2 (two corresponding images to solve affine model)
%        SetsX, SetsY (keypoints correspondence)
% bug: access out of boundary

load('../hw4_supp/tracked_points.mat');
addpath('../hw4_supp/images');
ImgfileDir='D:\Matlab_workspace\Homework\hw4_supp\images\';  %color
vistail= '.png';
frames=dir([ImgfileDir  '*' vistail] );
nFrame=length(frames);
% function ShiTomasiAffineVerification(image1, image2, SetsX, SetsY)
    image1 = imread('hotel.seq00.png');
    image2 = imread('hotel.seq10.png');
    pos_x1 = Xs(1, :); pos_y1 = Ys(1, :);
    pos_x2 = Xs(11, :); pos_y2 = Ys(11, :);
    
    dx = [-1 0 1];
    dy = [-1 0 1]';
    J_x = filter2(dx, image2, 'same');
    J_y = filter2(dy, image2, 'same');
    
    for index = 1:size(pos_x1, 2)
        T = zeros(6, 6);
        A = zeros(6, 1);
        for indx_win = -2:2
            for indy_win = -2:2
                x = pos_x1(index)+indx_win;
                y = pos_y1(index)+indy_win;
                gx = J_x(round(x), round(y));
                gy = J_y(round(x), round(y));
                t = [x^2*gx^2, x^2*gx*gy, x*y*gx^2, x*y*gx*gy, x*gx^2, x*gx*gy;...
                     x^2*gx*gy, x^2*gy^2, x*y*gx*gy, x*y*gy^2, x*gx*gy, x*gy^2;...
                     x*y*gx^2, x*y*gx*gy, y^2*gx^2, y^2*gx*gy, y*gx^2, y*gx*gy;...
                     x*y*gx*gy, x*y*gy^2, y^2*gx*gy, y^2*gy^2, y*gx*gy, y*gy^2;...
                     x*gx^2, x*gx*gy, y*gx^2, y*gx*gy, gx^2, gx*gy;...
                     x*gx*gy, x*gy^2, y*gx*gy, y*gy^2, gx*gy, gy^2];
                T = T + t;
                
                intensity_diff = double(image1(round(y), round(x))-image2(round(y), round(x)));
                a = intensity_diff * [x*gx; x*gy; y*gx; y*gy; gx; gy];
                A = A + a;
            end
        end
        
        z = inv(T) * A;
        D = [z(1), z(3); z(2), z(4)];
        d = [z(5); z(6)];
        A = eye(2, 2) + D;
        image_affined = zeros(43, 43);
        image_unaffined = zeros(43, 43);
        for indx_win = -21:21
            for indy_win = -21:21
                x = pos_x1(index)+indx_win;
                y = pos_y1(index)+indy_win;
                pos = [x; y];
                pos_affined = A*pos + d;
                if round(pos_affined(1)) > 640 || round(pos_affined(1)) <= 0 || round(pos_affined(2)) > 480 || round(pos_affined(2)) <= 0
                    break;
                end
                image_affined(indy_win+22, indx_win+22) = image2(round(pos_affined(2)), round(pos_affined(1)));
                image_unaffined(indy_win+22, indx_win+22) = image1(round(y), round(x));
            end
        end
        figure(1); imagesc(image_affined);
        figure(2); imagesc(image_unaffined);
        waitforbuttonpress;
    end
% end