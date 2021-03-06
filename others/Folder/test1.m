close all 
clc
clear all

N = 10; %use even number
degree = 1; 
x_trans = linspace(-1, 1, N);
c = abs(x_trans.^(-degree));
y_trans = c./sum(c, 'all');

initial = [-1,0];
final = [-5,-19];
distance = final-initial
magnitude = norm(distance);
yx = y_trans.*distance(:,1);
yy = y_trans.*distance(:,2);

x = [];
y = [];
a = initial(:,1);
b = initial(:,2);
for i=1:N
    x(i) = a+yx(i)
    a = x(i);
    y(i) = b+yy(i)
    b = y(i);
end

figure()
plot(x,y, 'o')