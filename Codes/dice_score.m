function out = dice_score(xyz1,xyz2)
[len1,~] = size(xyz1);
[len2,~] = size(xyz2);

shp = alphaShape(xyz1(:,1), xyz1(:,2), xyz1(:,3),'HoleThreshold',15); % create alphaShape
tf = inShape(shp,xyz2(:,1), xyz2(:,2), xyz2(:,3)); % test if points of A are inside B
Common_points = sum(tf);
out = 2*Common_points/(len1+len2);