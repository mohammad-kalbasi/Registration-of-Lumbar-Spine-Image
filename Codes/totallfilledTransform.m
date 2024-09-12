function PointCloutout = totallfilledTransform(V_moving,vx,vy,vz,movingCloud)
% creating point Cloud for all points(inside and edge)
xyz_boundry = movingCloud.Location;
minx = min(xyz_boundry(:,1));
miny = min(xyz_boundry(:,2));
minz = min(xyz_boundry(:,3));
V_moving(V_moving>0) = 1;
[x1,y1,z1] = size(V_moving);
lenlabel = sum(sum(sum(V_moving == 1)));
x_point = zeros(1,lenlabel);
y_point = zeros(1,lenlabel);
z_point = zeros(1,lenlabel);
counter = 1;
for i = 1: x1
    for j = 1:y1
        for k = 1:z1
            if V_moving(i,j,k) == 1
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
tform_fill = zeros(size(xyzPoints));

[len,~] = size(xyzPoints);


for i = 1:len
    tform_fill(i,1) = vx(xyzPoints(i,2) - miny+1, xyzPoints(i,1) - minx+1,xyzPoints(i,3) - minz+1);
    tform_fill(i,2) = vy(xyzPoints(i,2) - miny+1, xyzPoints(i,1) - minx+1,xyzPoints(i,3) - minz+1);
    tform_fill(i,3) = vz(xyzPoints(i,2) - miny+1, xyzPoints(i,1) - minx+1,xyzPoints(i,3) - minz+1);
end

movingCloud = pointCloud(xyzPoints);
PointCloutout = pctransform(movingCloud,tform_fill);
