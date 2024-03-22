clc;clear;close all
load("camera_params.mat")

sub = rossubscriber("/tracking","sensor_msgs/Image","DataFormat","struct");
i = 1;
figure(1)
while true
    msg = receive(sub);
    img = rosReadImage(msg);

    chk = [];
    img = undistortFisheyeImage(img,Params.Intrinsics);
    % gray = rgb2gray(img);
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

    if ~isempty(centers1)
        valid_ceneter1 = false(size(centers1,1),1);
        for j = 1:size(centers1,1)
            if all(chk(j,:)) && 50 < stats1.Area(j) && 10000 > stats1.Area(j)
                valid_ceneter1(j) = true;
                % viscircles(sq_plt_center,radii1);
            end
        end
        sq_plt_center = centers1(valid_ceneter1,:);
    end

    
    CC2 = bwconncomp(bw);
    stats2 = regionprops("table",CC2,"Centroid", ...
    "MajorAxisLength","MinorAxisLength","Area");
    centers2 = stats2.Centroid;
    diameters2 = mean([stats2.MajorAxisLength stats2.MinorAxisLength],2);
    
    if ~isempty(centers2)
        valid_ceneter2 = false(size(centers2,1),1);
        if  ~isempty(sq_plt_center)
            for j = 1:size(centers2,1)
                angle = atan2((centers2(j,2) - sq_plt_center(1,2)),(centers2(j,1) - sq_plt_center(1,1)));
                if (2.4 > angle && 2 < angle && 1000 < stats2.Area(j) && 7000 > stats2.Area(j)) ||...
                   (1.5 > angle && 0.7 < angle && 1000 < stats2.Area(j) && 7000 > stats2.Area(j))
                    valid_ceneter2(j) = true;
                    radii2 = diameters2(j)/2;
                    chassis_ceneter = centers2(j,:);
                    
                end
            end
        end
    end

    imshow(mask);
    if any(valid_ceneter1) && any(valid_ceneter2)
        delX = mean(diff(centers2(valid_ceneter2,1)));
        delY = mean(diff(centers2(valid_ceneter2,2)));
        if ~isempty(delX) && ~isempty(delY)
            distBetweenChassisCutouts(i,1) = sqrt(delX^2 + delY^2);
            i = i+1;
        end

        hold on
        viscircles([centers2(valid_ceneter2,:);sq_plt_center],[diameters2(valid_ceneter2)/2;diameters1(valid_ceneter1)/2],"Color","b");
        plot(centers2(valid_ceneter2,1),centers2(valid_ceneter2,2),"-r","LineWidth",2)
        hold off
    end

    % pause(0.02)
end
%%
clc;clear;close all
load("Data.mat")
min_dist = min(distBetweenChassisCutouts);
max_dist = max(distBetweenChassisCutouts);
final_dist_values = (distBetweenChassisCutouts - min_dist)/max_dist;
% height = [0:2:30 40 50];

figure(2)
plot(final_dist_values)
grid on
grid minor