function out = Hausdorff_score(xyz1, xyz2)
[len1, ~] = size(xyz1);
[len2, ~] = size(xyz2);

S1 = zeros(len1,1);
S2 = zeros(len2,1);
for i = 1:len1
    temp = xyz1(i,:);
    distance = sqrt(sum((xyz2 - temp).^2,2));
    S1(i) = min(distance);
end
for i = 1:len2
    temp = xyz2(i,:);
    distance = sqrt(sum((xyz1 - temp).^2,2));
    S2(i) = min(distance);
end
out = max([S1;S2]);
