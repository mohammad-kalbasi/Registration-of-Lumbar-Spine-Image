%%
clc;
clear;
%% initializing address
%moving image
address1 = 'Healthy_sample/00_mask.nii';
%fixed image
address2 = input("Enter Fixed image address\n", 's');
%% First parts are like method one 

V = niftiread(address1);
V_moving = V;
V_moving(V<20 | V > 24) = 0;
figure()
labelvolshow(V_moving)
figure()
BW = bwperim(V_moving);
volshow(BW)
title('volum of moving image ');
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

movingCloud_pre = pointCloud(xyzPoints);
movingCloudd_pre = pcdownsample(movingCloud_pre,'random',0.04);
figure()
pcshow(movingCloudd_pre)
title('downsampled point cloud of moving image');


%% for fixed image

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

%% now applying transform with Rigid transform this time

tform = pcregistercpd(movingCloudd_pre,fixedCloudd,'Transform','Rigid','MaxIterations',40);



%%
figure
pcshowpair(movingCloud_pre,fixedCloud,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds before registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%%
movingRigid= pctransform(movingCloud_pre,tform);
figure
pcshowpair(movingRigid,fixedCloudd,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after rigid registration')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')
xyzrigid = movingRigid.Location;
xyzrigid = round(xyzrigid);
movingRigid = pointCloud(xyzrigid);
movingRigidd= pctransform(movingCloudd_pre,tform);
xyzrigid = movingRigidd.Location;
xyzrigid = round(xyzrigid);
movingRigidd = pointCloud(xyzrigid);
figure
pcshowpair(movingRigid,fixedCloudd,'MarkerSize',50)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration with rigid transform (and rounding')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%% now applying non rigid transform

tform_nonrigid = pcregistercpd(movingRigidd,fixedCloudd,'Transform','Nonrigid','MaxIterations',40);



%% completing transform matrix

%interpolating transform for all points
[tform_totall,vx,vy,vz] = interpTransform(movingRigid,movingRigidd,tform_nonrigid);


%%
movingRegTotallpoints = pctransform(movingRigid,tform_totall);
figure
pcshowpair(movingRegTotallpoints,fixedCloud,'MarkerSize',10)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Point clouds after registration with non rigid transform')
legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
legend('Location','southoutside')

%% Part 2 calculating point that have intersection with each other
spinPointCloudTransform20 = movingspine_level_two(V_moving,tform,20,movingRigid,vx,vy,vz);
spinPointCloudTransform21 = movingspine_level_two(V_moving,tform,21,movingRigid,vx,vy,vz);
spinPointCloudTransform22 = movingspine_level_two(V_moving,tform,22,movingRigid,vx,vy,vz);
spinPointCloudTransform23 = movingspine_level_two(V_moving,tform,23,movingRigid,vx,vy,vz);
spinPointCloudTransform24 = movingspine_level_two(V_moving,tform,24,movingRigid,vx,vy,vz);
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

PointCloutout = totallfilledTransform_V2(V_moving,vx,vy,vz,movingRigid,tform);

PointCloudFull_fixed = PointCloudFull(V_fixed);

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