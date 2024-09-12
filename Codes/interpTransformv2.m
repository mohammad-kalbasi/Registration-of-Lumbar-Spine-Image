function [vx,vy,vz] = interpTransformv2(movingCloud,movingCloud2,movingCloudd,tform)
%% finding full tform transform
xyz = movingCloudd.Location;
xyz_boundry = movingCloud.Location;
xyz_boundry2 = movingCloud2.Location;
minx1 = min(xyz_boundry(:,1));
miny1 = min(xyz_boundry(:,2));
minz1 = min(xyz_boundry(:,3));

minx2 = min(xyz_boundry2(:,1));
miny2 = min(xyz_boundry2(:,2));
minz2 = min(xyz_boundry2(:,3));


maxx1 = max(xyz_boundry(:,1));
maxy1 = max(xyz_boundry(:,2));
maxz1 = max(xyz_boundry(:,3));

maxx2 = max(xyz_boundry2(:,1));
maxy2 = max(xyz_boundry2(:,2));
maxz2 = max(xyz_boundry2(:,3));

minx = min(minx1,minx2);
miny = min(miny1,miny2);
minz = min(minz1,minz2);

maxx = max(maxx1,maxx2);
maxy = max(maxy1,maxy2);
maxz = max(maxz1,maxz2);

x_vec = minx : 1 : maxx;
y_vec = miny : 1 : maxy;
z_vec = minz : 1 : maxz;

[X,Y,Z] = meshgrid(x_vec,y_vec,z_vec);

v1 = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,1),X,Y,Z,'linear');
v2 = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,2),X,Y,Z,'linear');
v3 = griddata(xyz(:,1),xyz(:,2),xyz(:,3),tform(:,3),X,Y,Z,'linear');



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
