%%
clc;
clear; 
%% initializing address
%moving image
address1 = 'Healthy_sample/00_mask.nii';
%fixed image
address2 = input("Enter Fixed image address\n", 's');
%% loading and creating pointCloud for each spine separately
V = niftiread(address1);
V_moving = V;
V_moving(V<20 | V>24) = 0;
V = niftiread(address2);
V_fixed = V;
V_fixed(V<20 | V>24) = 0;

PointCloudMoving20 = EdgePointCloud(V_moving,20);
PointCloudMoving21 = EdgePointCloud(V_moving,21);
PointCloudMoving22 = EdgePointCloud(V_moving,22);
PointCloudMoving23 = EdgePointCloud(V_moving,23);
PointCloudMoving24 = EdgePointCloud(V_moving,24);

PointClouddMoving20 = pcdownsample(PointCloudMoving20,'random',0.2);
PointClouddMoving21 = pcdownsample(PointCloudMoving21,'random',0.2);
PointClouddMoving22 = pcdownsample(PointCloudMoving22,'random',0.2);
PointClouddMoving23 = pcdownsample(PointCloudMoving23,'random',0.2);
PointClouddMoving24 = pcdownsample(PointCloudMoving24,'random',0.2);


PointCloudFixed20 = EdgePointCloud(V_fixed,20);
PointCloudFixed21 = EdgePointCloud(V_fixed,21);
PointCloudFixed22 = EdgePointCloud(V_fixed,22);
PointCloudFixed23 = EdgePointCloud(V_fixed,23);
PointCloudFixed24 = EdgePointCloud(V_fixed,24);

PointClouddFixed20 = pcdownsample(PointCloudFixed20,'random',0.2);
PointClouddFixed21 = pcdownsample(PointCloudFixed21,'random',0.2);
PointClouddFixed22 = pcdownsample(PointCloudFixed22,'random',0.2);
PointClouddFixed23 = pcdownsample(PointCloudFixed23,'random',0.2);
PointClouddFixed24 = pcdownsample(PointCloudFixed24,'random',0.2);

%% finding transforms for each set of pointClouds
tic
tform20 = pcregistercpd(PointClouddMoving20,PointClouddFixed20,'Transform','Nonrigid','MaxIterations',80);
toc
tform21 = pcregistercpd(PointClouddMoving21,PointClouddFixed21,'Transform','Nonrigid','MaxIterations',40);
tform22 = pcregistercpd(PointClouddMoving22,PointClouddFixed22,'Transform','Nonrigid','MaxIterations',40);
tform23 = pcregistercpd(PointClouddMoving23,PointClouddFixed23,'Transform','Nonrigid','MaxIterations',40);
tform24 = pcregistercpd(PointClouddMoving24,PointClouddFixed24,'Transform','Nonrigid','MaxIterations',40);


%% for 20
figure
pcshowpair(PointClouddMoving20,PointClouddFixed20,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%%
transformMoving20 = pctransform(PointClouddMoving20,tform20);
figure
pcshowpair(transformMoving20,PointClouddFixed20,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 20 after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%% for 21
figure
pcshowpair(PointClouddMoving21,PointClouddFixed21,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%%
transformMoving21 = pctransform(PointClouddMoving21,tform21);
figure
pcshowpair(transformMoving21,PointClouddFixed21,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 21 after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')


%%  for 22
figure
pcshowpair(PointClouddMoving22,PointClouddFixed22,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 22 before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%%
transformMoving22 = pctransform(PointClouddMoving22,tform22);
figure
pcshowpair(transformMoving22,PointClouddFixed22,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 22 after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')


%% for 23
figure
pcshowpair(PointClouddMoving23,PointClouddFixed23,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 23 before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%% 
transformMoving23 = pctransform(PointClouddMoving23,tform23);
figure
pcshowpair(transformMoving23,PointClouddFixed23,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 23 after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')


%% for 24
figure
pcshowpair(PointClouddMoving24,PointClouddFixed24,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%%
transformMoving24 = pctransform(PointClouddMoving24,tform24);
figure
pcshowpair(transformMoving24,PointClouddFixed24,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds label 21 after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')


%% creating a totall pointCloud and then interpolating and finding full deformation field
tform = [tform20;tform21;tform22;tform23;tform24];
xyz1d = PointClouddMoving20.Location;
xyz2d = PointClouddMoving21.Location;
xyz3d = PointClouddMoving22.Location;
xyz4d = PointClouddMoving23.Location;
xyz5d = PointClouddMoving24.Location;
xyzd = [xyz1d;xyz2d;xyz3d;xyz4d;xyz5d];
movingCloudd = pointCloud(xyzd);
xyz1 = PointCloudMoving20.Location;
xyz2 = PointCloudMoving21.Location;
xyz3 = PointCloudMoving22.Location;
xyz4 = PointCloudMoving23.Location;
xyz5 = PointCloudMoving24.Location;
xyz = [xyz1;xyz2;xyz3;xyz4;xyz5];
movingCloud = pointCloud(xyz);
[tform_totall,vx,vy,vz] = interpTransform(movingCloud,movingCloudd,tform);
%% creating fixed Cloud

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
%% applying transform on totall points

movingRegTotallpoints = pctransform(movingCloud,tform_totall);
figure
pcshowpair(movingRegTotallpoints,fixedCloud,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')


%% Part 2 calculating point that have intersection with each other
spinPointCloudTransform20 = movingspine(V_moving,20,movingCloud,vx,vy,vz);
spinPointCloudTransform21 = movingspine(V_moving,21,movingCloud,vx,vy,vz);
spinPointCloudTransform22 = movingspine(V_moving,22,movingCloud,vx,vy,vz);
spinPointCloudTransform23 = movingspine(V_moving,23,movingCloud,vx,vy,vz);
spinPointCloudTransform24 = movingspine(V_moving,24,movingCloud,vx,vy,vz);

xyz1 = spinPointCloudTransform20.Location;
xyz2 = spinPointCloudTransform21.Location;
intersect = zeros(4,1);
% calculating intersection points
bad_points = 0;
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
%%
%% calculating point
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
log_detmat = log10(abs(det_mat));
neg_pos_rato = sum(sum(sum(log_detmat<0)))/sizedet;
 disp(['number of negative elements of log-determinant divided by number of elements = ',num2str(neg_pos_rato)]);