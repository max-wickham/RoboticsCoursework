model = KinematicModel();
points = model.positions(10,10,10,0)
% x = points[0;]
x = points(:,1)
y = points(:,2)
z = points(:,3)
plot3(x,y,z);
