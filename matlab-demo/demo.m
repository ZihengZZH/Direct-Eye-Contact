% Quick Program to demo the use of projectPoints
% Also quick and interactive demo for C++ implementation of view synthesis
% SOME BUGS: OPPOSITE POSE BETWEEN VIRTUAL CAMERA AND PROJECTED IMAGE

%% generate a set of 3d points

% random points for testing
%{
z = peaks;
x = repmat(1:size(z,1),size(z,1),1);
y = x';
c = z - min(z(:));
c = c./max(c(:));
c = round(255*c) + 1;
cmap = colormap(jet(256));
c = cmap(c,:);

points = [x(:),y(:),z(:),c];
%}


% facial points reading from file
fidin = fopen('face.xyz', 'rt');
A = textscan(fidin, '%f %f %f %u %u %u');

points = [];
points(:,1) = A{1};
points(:,2) = A{2};
points(:,3) = A{3};
points(:,4) = A{4}; % mono-colour


%% setup 

%setup camera with focal length 675, centre 320,240 (resolution 640*480)
cam = [675,0,320;0,675,240;0,0,1];

%setup image (large enough for projection)
imageSize = [3000,3000];

%inverse (change this variable to see opposite pose)
inv = 1;

%create a tform matrix
angles = inv*[0,0,-25]*pi/180;
position = inv*[0,0.5,0.];
tform = eye(4);
tform(1:3,1:3) = angle2dcm(angles(1),angles(2),angles(3));
tform(1:3,4) = position;

%camera location and pose
%BUT wired for the opposite as projected image
orientation = tform(1:3,1:3)';
location = position * orientation;

%add no lens distortion
dist = [0.,0.0];

%project the points into image coordinates
[projected, valid] = projectPoints(points, cam, tform, dist, imageSize,true);
projected = projected(valid,:);

%show the projection
subplot(1,2,1);
scatter3(A{1}, A{2}, A{3},20, [A{4}],'fill');
hold on
scatter3(points(:,1),points(:,2),points(:,3),20, points(:,4),'fill');
cam = plotCamera('Location', location, 'Orientation', orientation, 'Size', 0.05);
hold off
axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
title('Original Points');
grid;

subplot(1,2,2);
scatter(projected(:,1),projected(:,2),20,projected(:,3),'fill');
axis equal;
title('Points projected with camera model');

