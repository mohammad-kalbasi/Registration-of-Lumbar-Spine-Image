function PointCloudOut = EdgePointCloud(V,label)
V(V~= label) = 0;
BW = bwperim(V);

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

PointCloudOut = pointCloud(xyzPoints);