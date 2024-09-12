function showZslice(V,slice)
V = V(:,:,slice);
[m,n] = size(V);
out = zeros(m,n,3);
for i = 1:m
    for j = 1:n
        if(V(i,j) == 20)
            out(m,n,1) = 255;
        end
         if(V(i,j) == 21)
            out(m,n,2) = 255;
         end
         if(V(i,j) == 22)
            out(m,n,3) = 255;
         end
         if(V(i,j) == 23)
            out(m,n,1) = 124;
            out(m,n,2) = 125;
         end
         if(V(i,j) == 24)
            out(m,n,1) = 255;
            out(m,n,3) = 125;
        end
    end
end
figure()
imshow(out);