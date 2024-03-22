%% Area of shadow on camera sensor
clc;clear;close all
load("camera_params.mat")
imd = imageDatastore("/home/sagar/ROS/src/ros_bag/images/new_tracking_record/");

lr = 0.034;
Ls = 0.083; % edge length of the sq. plate
As = Ls^2; % Area of the sq. plate (m^2)
ds = 0.092; % Effective arm length of the sq. plate
hl = 0.2; % Height of the light source from deck (m)
f = 0.00083; % Focal length of the lens (m)
p = 3e-6; % Pixelpitch (m)

alpha = 1 + ((Ls + 2*ds)/(2*hl))^2;
beta = (4*lr*ds + Ls^2 + 2*Ls*lr + 2*Ls*ds)/(2*hl);
gamma = (Ls/2 + lr)^2;

for i = 1:length(imd.Files)
    img = readimage(imd,i);
    % Undistorting the original image
    img = undistortFisheyeImage(img,Params.Intrinsics);
    % Converting to gray scale
    gray = rgb2gray(img);
    % Thresholding
    T = adaptthresh(gray,"Statistic","gaussian");
    bw = imbinarize(gray,T);
    mask = ~bw;
    figure(1)
    imshowpair(img,mask,"montage")
    % Finding number of connective components
    CC = bwconncomp(mask);
    % Finding the number of pixels in the second-largest region
    stats = regionprops(CC,'Area');
    stats = vertcat(stats.Area);
    stats = sort(stats,"descend");
    % Finding the area of the region
    area = stats(2)*(p^2);
    
    % Calculating the height of the UAV
    h(:,i) = roots([(alpha*area^2 - (f*As)^2)...
                (beta*area^2 - 2*hl*(f*As)^2)...
                (gamma*area^2 - (f*As*hl)^2)]);
end

fps = 30;
t = linspace(0,length(imd.Files)/fps,length(imd.Files));

figure(2)
plot(t,h)
ylabel("Height(m)")
xlabel("Time(sec)")
grid on
grid minor
%%
clc;clear;close all
load("camera_params.mat")
As = 0.06*0.06; % Area of the sq. plate (m^2)
hl = 0.2; % Height of the light source from deck (m)
f = 0.00083; % Focal length of the lens (m)
p = 3e-6; % Pixelpitch (m)
i = 1;
sub = rossubscriber("/tracking","sensor_msgs/Image","DataFormat","struct");
while true
    msg = receive(sub); 
    img = rosReadImage(msg);

    % Undistorting the original image
    img = undistortFisheyeImage(img,Params.Intrinsics);
    % Converting to gray scale
    % gray = rgb2gray(img);
    % Thresholding
    T = adaptthresh(img,"Statistic","gaussian");
    bw = imbinarize(img,T);
    mask = ~bw;
    figure(1)
    imshowpair(img,mask,"montage")
    % Finding number of connective components
    CC = bwconncomp(mask);
    % Finding the number of pixels in the second-largest region
    stats = regionprops(CC,'Area');
    stats = vertcat(stats.Area);
    stats = sort(stats,"descend");
    % Finding the area of the region
    area = stats(2)*(p^2)*1000*cosd(45);
    % Calculating the height of the UAV
    F = (As*f*hl)/(area*hl - As*f) - 0.065;
    h = abs(sqrt(F^2 - ds^2))

    i = i+1;
end
%%
clc;clear;close all
load("camera_params.mat")
lr = 0.03;
Ls = 0.038; % edge length of the sq. plate
As = Ls^2; % Area of the sq. plate (m^2)
ds = 0.08; % Effective arm length of the sq. plate
hl = 0.2; % Height of the light source from deck (m)
f = 0.00083; % Focal length of the lens (m)
p = 3e-6; % Pixelpitch (m)

alpha = 1 + ((Ls + 2*ds)/(2*hl))^2;
beta = (4*lr*ds + Ls^2 + 2*Ls*lr + 2*Ls*ds)/(2*hl);
gamma = (Ls/2 + lr)^2;
i = 1;

sub = rossubscriber("/tracking","sensor_msgs/Image","DataFormat","struct");
while true
    msg = receive(sub);
    img = rosReadImage(msg);

    % Undistorting the original image
    img = undistortFisheyeImage(img,Params.Intrinsics);
    % Converting to gray scale
    % gray = rgb2gray(img);
    % Thresholding
    T = adaptthresh(img,"Statistic","gaussian");
    bw = imbinarize(img,T);
    mask = ~bw;
    figure(1)
    imshowpair(img,mask,"montage")
    % Finding number of connective components
    CC = bwconncomp(mask);
    % Finding the number of pixels in the second-largest region
    stats = regionprops(CC,'Area');
    stats = vertcat(stats.Area);
    stats = sort(stats,"descend");
    % Finding the area of the region
    area = stats(2)*(p^2)*cosd(45)*1000;

    % Calculating the height of the UAV
    h = roots([(alpha*(area*hl)^2 - (f*As)^22)...
                (beta*(area*hl)^2 - 2*hl*(f*As)^2)...
                (gamma*(area*hl)^2 - (f*As*hl)^2)]);
    h = abs(h)
    % H(i,:) = [abs(h(1,1)) abs(h(2,1))];
    i = i+1;
end
%%
clc;clear;close all
load("camera_params.mat")
imd = imageDatastore("/home/sagar/ROS/src/ros_bag/images/new_tracking_record/");

lr = 0.03;
L_sq = 0.038; % edge length of the sq. plate
A_sq = L_sq^2; % Area of the sq. plate (m^2)
ds = 0.08; % Effective arm length of the sq. plate
hl = 0.2; % Height of the light source from deck (m)
f = 0.00083; % Focal length of the lens (m)
p = 3e-6; % Pixelpitch (m)

h = 0:0.01:0.5;
d = lr + (h'*ds)/hl;
A_shadow = A_sq*(h'+hl)/hl;
L_shadow = L_sq*(h'+hl)/hl;
F = sqrt(d.^2 + h'.^2);
a_sensor = A_shadow*f./F;

sub = rossubscriber("/tracking","sensor_msgs/Image","DataFormat","struct");
while true
    msg = receive(sub);
    img = rosReadImage(msg);

    % Undistorting the original image
    img = undistortFisheyeImage(img,Params.Intrinsics);
    % Converting to gray scale
    % gray = rgb2gray(img);
    % Thresholding
    T = adaptthresh(img,"Statistic","gaussian");
    bw = imbinarize(img,T);
    mask = ~bw;
    figure(1)
    imshowpair(img,mask,"montage")
    % Finding number of connective components
    CC = bwconncomp(mask);
    % Finding the number of pixels in the second-largest region
    stats = regionprops(CC,'Area');
    stats = vertcat(stats.Area);
    stats = sort(stats,"descend");
    % Finding the area of the region
    area = stats(2)*(p^2)*cosd(45)*1000;
end