%%
clc;
clear;
%% initializing address
%moving image
address1 = 'Healthy_sample/00_mask.nii';
%fixed image
address2 = input("Enter Fixed image address\n", 's');
%% Loading healthy sample

V = niftiread(address1);
V_moving = V;
V_moving(V<20 | V>24) = 0;
figure()
labelvolshow(V_moving)
figure()
BW = bwperim(V_moving); % edge detection for finding pointCloud
volshow(BW)
title('volum of moving image ');
%%
len = sum(sum(sum(BW == 1)));
x_point = zeros(1,len);
y_point = zeros(1,len);
z_point = zeros(1,len);
counter = 1;
[x1,y1,z1] = size(BW); 
%Extracting positions of points
for i = 1: x1
    for j = 1:y1
        for k = 1:z1
            if BW(i,j,k) == 1
                x_point(counter) = i;
                y_point(counter) = j;
                z_point(counter) = k;
                counter = counter + 1;
            end
        end
    end
end


xyzPoints = zeros(length(x_point),3);
xyzPoints(:,1) = x_point;
xyzPoints(:,2) = y_point;
xyzPoints(:,3) = z_point;
%creating CloudPoint
movingCloud = pointCloud(xyzPoints);
movingCloudd = pcdownsample(movingCloud,'random',0.04);
figure()
pcshow(movingCloudd)
title('downsampled point cloud of moving image');


%% for fixed image
% like previous part for fixed image
V = niftiread(address2);
V_fixed = V;
V_fixed(V<20 | V > 24) = 0;
figure()
labelvolshow(V_fixed)
figure()
BW = bwperim(V_fixed);
volshow(BW)
%%
len = sum(sum(sum(BW == 1)));
x_point = zeros(1,len);
y_point = zeros(1,len);
z_point = zeros(1,len);
counter = 1;
[x1,y1,z1] = size(BW);
for i = 1: x1
    for j = 1:y1
        for k = 1:z1
            if BW(i,j,k) == 1
                x_point(counter) = i;
                y_point(counter) = j;
                z_point(counter) = k;
                counter = counter + 1;
            end
        end
    end
end


xyzPoints = zeros(length(x_point),3);
xyzPoints(:,1) = x_point;
xyzPoints(:,2) = y_point;
xyzPoints(:,3) = z_point;

fixedCloud = pointCloud(xyzPoints);

fixedCloudd = pcdownsample(fixedCloud,'random',0.04);
figure()
pcshow(fixedCloudd)
title('downsampled point cloud of fixed image');

%% registeration Using CPD (Non Rigid)
tform = pcregistercpd(movingCloudd,fixedCloudd,'Transform','Nonrigid','MaxIterations',40);

%% Plotting them befor registration
figure 
pcshowpair(movingCloud,fixedCloud,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%% plotting them after registration
movingRegcpd = pctransform(movingCloudd,tform);
figure
pcshowpair(movingRegcpd,fixedCloudd,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration with rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')


%% completing transform matrix


[tform_totall,vx,vy,vz] = interpTransform(movingCloud,movingCloudd,tform);
% interpolating and finding full deformation field and then compliting T
% transform for all points


%% apllying transform to all points
movingRegTotallpoints = pctransform(movingCloud,tform_totall);
figure
pcshowpair(movingRegTotallpoints,fixedCloud,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%% Part 2 calculating intersection of points
spinPointCloudTransform20 = movingspine(V_moving,20,movingCloud,vx,vy,vz); %creating and transforming each point cloud for each spine
spinPointCloudTransform21 = movingspine(V_moving,21,movingCloud,vx,vy,vz);
spinPointCloudTransform22 = movingspine(V_moving,22,movingCloud,vx,vy,vz);
spinPointCloudTransform23 = movingspine(V_moving,23,movingCloud,vx,vy,vz);
spinPointCloudTransform24 = movingspine(V_moving,24,movingCloud,vx,vy,vz);
bad_points = 0;
xyz1 = spinPointCloudTransform20.Location;
xyz2 = spinPointCloudTransform21.Location;
intersect = zeros(4,1);
% calculating intersection points
shp = alphaShape(xyz1(:,1), xyz1(:,2), xyz1(:,3),'HoleThreshold',15); % create alphaShape
tf = inShape(shp,xyz2(:,1), xyz2(:,2), xyz2(:,3)); % test if points of A are inside B
bad_points = bad_points + sum(tf);
intersect(1) = sum(tf);
xyz1 = spinPointCloudTransform21.Location;
xyz2 = spinPointCloudTransform22.Location;

shp = alphaShape(xyz1(:,1), xyz1(:,2), xyz1(:,3),'HoleThreshold',15); % create alphaShape
tf = inShape(shp,xyz2(:,1), xyz2(:,2), xyz2(:,3)); % test if points of A are inside B
bad_points = bad_points + sum(tf);
intersect(2) = sum(tf);

xyz1 = spinPointCloudTransform22.Location;
xyz2 = spinPointCloudTransform23.Location;

shp = alphaShape(xyz1(:,1), xyz1(:,2), xyz1(:,3),'HoleThreshold',15); % create alphaShape
tf = inShape(shp,xyz2(:,1), xyz2(:,2), xyz2(:,3)); % test if points of A are inside B
bad_points = bad_points + sum(tf);
intersect(3) = sum(tf);

xyz1 = spinPointCloudTransform23.Location;
xyz2 = spinPointCloudTransform24.Location;

shp = alphaShape(xyz1(:,1), xyz1(:,2), xyz1(:,3),'HoleThreshold',15); % create alphaShape
tf = inShape(shp,xyz2(:,1), xyz2(:,2), xyz2(:,3)); % test if points of A are inside B
bad_points = bad_points + sum(tf);
intersect(4) = sum(tf);




disp(['number of points that have intersection with other parts is equal to = ',num2str(bad_points)]);
%% creating full point Cloud for transformed image

PointCloutout = totallfilledTransform(V_moving,vx,vy,vz,movingCloud);
PointCloudFull_fixed = PointCloudFull(V_fixed);

%% calculatin Points
dicescore = dice_score(PointCloutout.Location,PointCloudFull_fixed.Location);
disp(['dice score output = ',num2str(dicescore)]);

Hausdorffscore = Hausdorff_score(movingRegTotallpoints.Location, fixedCloud.Location);
disp(['Hausdorff score output = ',num2str(Hausdorffscore)]);

 ASscore = Average_Surface_score(movingRegTotallpoints.Location, fixedCloud.Location);
 disp(['Average Surface score output = ',num2str(ASscore)]);
 
 %% calculating determinant of jacobian
 det_mat =  jacob(vx,vy,vz);
 [detsx,detsy,detsz] = size(det_mat);
 sizedet = detsx*detsy*detsz;
 neg_pos_rato = sum(sum(sum(det_mat<0)))/sizedet;
 disp(['number of negative elements of determinant divided by number of elements = ',num2str(neg_pos_rato)]);
%% Log determinant of jacobian
log_detmat = log10(det_mat);
neg_pos_rato = sum(sum(sum(log_detmat<0)))/sizedet;
 disp(['number of negative elements of log-determinant divided by number of elements = ',num2str(neg_pos_rato)]);