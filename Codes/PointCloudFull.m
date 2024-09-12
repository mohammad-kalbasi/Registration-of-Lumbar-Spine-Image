function out = PointCloudFull(V)
V(V>0) = 1;
lenlabel = sum(sum(sum(V == 1)));
x_point = zeros(1,lenlabel);
y_point = zeros(1,lenlabel);
z_point = zeros(1,lenlabel);
[x1,y1,z1] = size(V);

counter = 1;
for i = 1: x1
    for j = 1:y1
        for k = 1:z1
            if V(i,j,k) == 1
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
out = pointCloud(xyzPoints);
