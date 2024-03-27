clc;clear;
load("camera_params.mat")
imd = imageDatastore("*.jpg");
figure(1)
for i = 1:length(imd.Files)
    chk = [];
    img = readimage(imd,i);
    img = undistortFisheyeImage(img,Params.Intrinsics);
    gray = rgb2gray(img);
    T = adaptthresh(gray,"Statistic","gaussian");
    bw = imbinarize(gray,T);

    mask = ~bw;
    CC1 = bwconncomp(mask);
    stats1 = regionprops("table",CC1,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Area");
    centers1 = stats1.Centroid;
    chk(:,1) = 310 < centers1(:,1);
    chk(:,2) = 325 > centers1(:,1);
    diameters1 = mean([stats1.MajorAxisLength stats1.MinorAxisLength],2);
    valid_ceneter1 = false(size(centers1,1));
    for j = 1:size(centers1,1)
        if all(chk(j,:)) && 50 < stats1.Area(j) && 10000 > stats1.Area(j)
            valid_ceneter1(j) = true;
            % viscircles(sq_plt_center,radii1);
        end
    end
    sq_plt_center = centers1(valid_ceneter1,:);

    
    CC2 = bwconncomp(bw);
    stats2 = regionprops("table",CC2,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Area");
    centers2 = stats2.Centroid;
    diameters2 = mean([stats2.MajorAxisLength stats2.MinorAxisLength],2);
    
    valid_ceneter2 = false(size(centers2,1));
    for j = 1:size(centers2,1)
        angle = atan2((centers2(j,2) - sq_plt_center(1,2)),(centers2(j,1) - sq_plt_center(1,1)));
        if (2.4 > angle && 2 < angle && 1500 < stats2.Area(j) && 7000 > stats2.Area(j)) ||...
           (1.5 > angle && 0.7 < angle && 1500 < stats2.Area(j) && 7000 > stats2.Area(j))
            valid_ceneter2(j) = true;
            radii2 = diameters2(j)/2;
            chassis_ceneter = centers2(j,:);
            
        end
    end
    delX = diff(centers2(valid_ceneter2,1));
    delY = diff(centers2(valid_ceneter2,2));
    distBetweenChassisCutouts(i,1) = sqrt(delX^2 + delY^2);

    imshow(mask);
    hold on
    viscircles(centers2(valid_ceneter2,:),diameters2(valid_ceneter2)/2,"Color","b");
    plot(centers2(valid_ceneter2,1),centers2(valid_ceneter2,2),"-r","LineWidth",2)
    hold off

    % frame = getframe(figure(1));
    % file = frame2im(frame);
    % imwrite(file,"img_"+num2str(i)+".png")
    pause(0.02)
end

min_dist = min(distBetweenChassisCutouts);
max_dist = max(distBetweenChassisCutouts);
final_dist_values = (distBetweenChassisCutouts - min_dist)/max_dist;
height = [0:2:30 40 50]';

% figure(1)
% plot(height,final_dist_values)
% xlabel("Height")
% ylabel("Distance between the two chassis cutouts (Normalized)")
% grid on
% grid minor
%% Spline
clc;
% Example data points (replace with your actual data)
x = height;
y = final_dist_values;
query_points = (0:0.1:50)';
% Perform spline interpolation
spline_curve = spline(x, y, query_points);

% Plot the original data points and the fitted spline curve
plot(x, y, 'bo');  % Plot data points
hold on;
plot(query_points,spline_curve, 'r-');  % Plot fitted spline curve
xlabel('x');
ylabel('y');
title('Fitted Spline Curve');
legend('Data Points', 'Fitted Spline Curve');
grid on;
hold off;
%% Test
clc;clear;close all
load("camera_params.mat")
load("Data.mat")
sub1 = rossubscriber("/tracking","sensor_msgs/Image","DataFormat","struct");
sub2 = rossubscriber("/qvio/pose", "geometry_msgs/PoseStamped", "DataFormat", "struct");
qvio_height = [];
i = 1;
figure(1)
pause(5)
disp('Starting...')

while rosshutdown
    chk = [];
    msg = receive(sub1);
    img = rosReadImage(msg);
    img = undistortFisheyeImage(img,Params.Intrinsics);
    T = adaptthresh(img,"Statistic","gaussian");
    bw = imbinarize(img,T);

    mask = ~bw;
    CC1 = bwconncomp(mask);
    stats1 = regionprops("table",CC1,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Area");
    centers1 = stats1.Centroid;
    chk(:,1) = 310 < centers1(:,1);
    chk(:,2) = 325 > centers1(:,1);
    diameters1 = mean([stats1.MajorAxisLength stats1.MinorAxisLength],2);
    valid_ceneter1 = false(size(centers1,1));
    for j = 1:size(centers1,1)
        if all(chk(j,:)) && 50 < stats1.Area(j) && 10000 > stats1.Area(j)
            valid_ceneter1(j) = true;
        end
    end
    sq_plt_center = centers1(valid_ceneter1,:);

    
    CC2 = bwconncomp(bw);
    stats2 = regionprops("table",CC2,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Area");
    centers2 = stats2.Centroid;
    diameters2 = mean([stats2.MajorAxisLength stats2.MinorAxisLength],2);
    
    valid_ceneter2 = false(size(centers2,1));
    for j = 1:size(centers2,1)
        angle = atan2((centers2(j,2) - sq_plt_center(1,2)),(centers2(j,1) - sq_plt_center(1,1)));
        if (2.4 > angle && 2 < angle && 1500 < stats2.Area(j) && 7000 > stats2.Area(j)) ||...
           (1.5 > angle && 0.7 < angle && 1500 < stats2.Area(j) && 7000 > stats2.Area(j))
            valid_ceneter2(j) = true;
            radii2 = diameters2(j)/2;
            chassis_ceneter = centers2(j,:);
            
        end
    end
    delX = diff(centers2(valid_ceneter2,1));
    delY = diff(centers2(valid_ceneter2,2));
    distBetweenChassisCutouts = sqrt(delX^2 + delY^2);
    normalized_dist(i) = (distBetweenChassisCutouts - min_dist)/max_dist;
    height(i) = spline(spline_curve,query_points,normalized_dist(i));
    fprintf("Current height: %.4f\n",height(i))

    if ~isempty(sub2.LatestMessage)
        qvio_height(i,1) = sub2.LatestMessage.Pose.Position.Z;
    else
        qvio_height(i,1) = NaN;
    end

    imshow(mask);
    hold on
    viscircles(centers2(valid_ceneter2,:),diameters2(valid_ceneter2)/2,"Color","b");
    plot(centers2(valid_ceneter2,1),centers2(valid_ceneter2,2),"-r","LineWidth",2)
    hold off
    i = i+1;
end
%%
plot(height,normalized_dist,'-ok');
xlabel('Normalized feature values');
ylabel('Height');
title('UAV height vs Distance of chassis cutout');
grid on;
grid minor;