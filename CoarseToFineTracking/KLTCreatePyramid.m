%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Create Pyramid
% image1 = imread('../hw4_supp/images/hotel.seq00.png');
% subsampling = 2;
% nlevels=4;
    
function [outimage] = KLTCreatePyramid(image1, subsampling, nlevels)
    if subsampling ~= 2 && subsampling ~= 4 && subsampling ~= 8 &&...
            subsampling ~= 16 && subsampling ~= 32
        return;
    end
    ncols = size(image1, 2);
    nrows = size(image1, 1);
    outimage = cell(1, nlevels);
    outimage{1} = image1;
    for index = 1:nlevels-1
        image = image1(1:subsampling:nrows, 1:subsampling:ncols);
        ncols = ncols / subsampling;
        nrows = nrows / subsampling;
        outimage{index+1} = image;
    end
end