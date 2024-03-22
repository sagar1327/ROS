clc;clear;
load("camera_params.mat")
imd = imageDatastore("*.jpg");

for i = 1:length(imd.Files)
    img = readimage(imd,i);
    img = undistortFisheyeImage(img,Params.Intrinsics);
    gray = rgb2gray(img);
    T = adaptthresh(gray,"Statistic","gaussian");
    bw = imbinarize(gray,T);
    mask = ~bw;
    CC = bwconncomp(mask);
    % Finding the number of pixels in the second-largest region
    stats = regionprops(CC,'Area');
    stats = vertcat(stats.Area);
    stats = sort(stats,"descend");
    Area(i,1) = stats(2);
    % imshow(mask)
end
min_area = min(Area);
max_area = max(Area);
Area = (Area - min_area)/max_area;
height = [0:2:30 40 50];

figure(1)
plot(height,Area)
grid on
grid minor