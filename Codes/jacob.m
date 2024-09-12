function out = jacob(vx,vy,vz)

% source of this code:
% https://www.mathworks.com/matlabcentral/fileexchange/39194-diffeomorphic-log-demons-image-registration
[gx_y,gx_x,gx_z] = gradient(vx);
[gy_y,gy_x,gy_z] = gradient(vy);
[gz_y,gz_x,gz_z] = gradient(vz);

% Add identity
gx_x = gx_x + 1;
gy_y = gy_y + 1;
gz_z = gz_z + 1;

% Determinant
out = gx_x.*gy_y.*gz_z + ...
        gy_x.*gz_y.*gx_z + ...
        gz_x.*gx_y.*gy_z - ...
        gz_x.*gy_y.*gx_z - ...
        gy_x.*gx_y.*gz_z - ...
        gx_x.*gz_y.*gy_z;
end



