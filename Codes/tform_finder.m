function tform_totall = tform_finder(movingCloud,v1,v2,v3,movingCloud2);
% creating T transform based on deformation field
xyz_boundry = movingCloud.Location;
xyz_boundry2 = movingCloud2.Location;
minx1 = min(xyz_boundry(:,1));
miny1 = min(xyz_boundry(:,2));
minz1 = min(xyz_boundry(:,3));

minx2 = min(xyz_boundry2(:,1));
miny2 = min(xyz_boundry2(:,2));
minz2 = min(xyz_boundry2(:,3));

minx = min(minx1,minx2);
miny = min(miny1,miny2);
minz = min(minz1,minz2);
xyz_boundry = movingCloud.Location;
tform_totall = zeros(size(xyz_boundry));
[len,~] = size(xyz_boundry);

for i = 1:len
    tform_totall(i,1) = v1(xyz_boundry(i,2) - miny+1, xyz_boundry(i,1) - minx+1,xyz_boundry(i,3) - minz+1);
    tform_totall(i,2) = v2(xyz_boundry(i,2) - miny+1, xyz_boundry(i,1) - minx+1,xyz_boundry(i,3) - minz+1);
    tform_totall(i,3) = v3(xyz_boundry(i,2) - miny+1, xyz_boundry(i,1) - minx+1,xyz_boundry(i,3) - minz+1);
end