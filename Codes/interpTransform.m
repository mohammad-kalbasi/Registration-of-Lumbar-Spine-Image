function [tform_totall,vx,vy,vz] = interpTransform(movingCloud,movingCloudd,tform)
%% finding full tform transform
% creating boundry
xyz = movingCloudd.Location;
xyz_boundry = movingCloud.Location;
x_vec = min(xyz_boundry(:,1)) : 1 : max(xyz_boundry(:,1));
y_vec = min(xyz_boundry(:,2)) : 1 : max(xyz_boundry(:,2));
z_vec = min(xyz_boundry(:,3)) : 1 : max(xyz_boundry(:,3));

[X,Y,Z] = meshgrid(x_vec,y_vec,z_vec);
%interpolation
v1 = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,1),X,Y,Z,'linear');
v2 = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,2),X,Y,Z,'linear');
v3 = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,3),X,Y,Z,'linear');


% second method of interpolation to fill Nan points
v1_temp = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,1),X,Y,Z,'nearest');
v2_temp = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,2),X,Y,Z,'nearest');
v3_temp = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,3),X,Y,Z,'nearest');

[row, col] = find(isnan(v1));
for i = 1:length(row)
    v1(row(i),col(i)) = v1_temp(row(i),col(i));
end

[row, col] = find(isnan(v2));
for i = 1:length(row)
    v2(row(i),col(i)) = v2_temp(row(i),col(i));
end



[row, col] = find(isnan(v3));
for i = 1:length(row)
    v3(row(i),col(i)) = v3_temp(row(i),col(i));
end
vx = v1;
vy = v2;
vz = v3;
tform_totall = zeros(size(xyz_boundry));
[len,~] = size(xyz_boundry);
minx = min(xyz_boundry(:,1));
miny = min(xyz_boundry(:,2));
minz = min(xyz_boundry(:,3));
% finding transformation for all points
for i = 1:len
    tform_totall(i,1) = v1(xyz_boundry(i,2) - miny+1, xyz_boundry(i,1) - minx+1,xyz_boundry(i,3) - minz+1);
    tform_totall(i,2) = v2(xyz_boundry(i,2) - miny+1, xyz_boundry(i,1) - minx+1,xyz_boundry(i,3) - minz+1);
    tform_totall(i,3) = v3(xyz_boundry(i,2) - miny+1, xyz_boundry(i,1) - minx+1,xyz_boundry(i,3) - minz+1);
end