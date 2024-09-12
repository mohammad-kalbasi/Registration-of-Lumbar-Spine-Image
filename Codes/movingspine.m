function spinPointCloudTransform = movingspine(V_moving,label,movingCloud,vx,vy,vz)
% extracting spine poin cloud and performing transformation on it
xyz_boundry = movingCloud.Location;
minx = min(xyz_boundry(:,1));
miny = min(xyz_boundry(:,2));
minz = min(xyz_boundry(:,3));

V_label = V_moving;
V_label(V_moving ~= label) = 0;
[x1,y1,z1] = size(V_label);
lenlabel = sum(sum(sum(V_label == label)));
x_pointlabel = zeros(1,lenlabel);
y_pointlabel = zeros(1,lenlabel);
z_pointlabel = zeros(1,lenlabel);
counter = 1;
for i = 1: x1
    for j = 1:y1
        for k = 1:z1
            if V_label(i,j,k) == label
                x_pointlabel(counter) = i;
                y_pointlabel(counter) = j;
                z_pointlabel(counter) = k;
                counter = counter + 1;
            end
        end
    end
end
xyzPointslabel = zeros(length(x_pointlabel),3);
xyzPointslabel(:,1) = x_pointlabel;
xyzPointslabel(:,2) = y_pointlabel;
xyzPointslabel(:,3) = z_pointlabel;
tform_label = zeros(size(xyzPointslabel));

[len,~] = size(xyzPointslabel);


for i = 1:len
    tform_label(i,1) = vx(xyzPointslabel(i,2) - miny+1, xyzPointslabel(i,1) - minx+1,xyzPointslabel(i,3) - minz+1);
    tform_label(i,2) = vy(xyzPointslabel(i,2) - miny+1, xyzPointslabel(i,1) - minx+1,xyzPointslabel(i,3) - minz+1);
    tform_label(i,3) = vz(xyzPointslabel(i,2) - miny+1, xyzPointslabel(i,1) - minx+1,xyzPointslabel(i,3) - minz+1);
end


movingCloudlabel = pointCloud(xyzPointslabel);
spinPointCloudTransform = pctransform(movingCloudlabel,tform_label);
