%generate the target plot
[X,Y] = meshgrid(-2:.2:2, -2:.2:2);
Z = X .* exp(-X.^2 - Y.^2);

figure(1)
mesh(X,Y,Z);

%generate an animation of how the robot will follow the trajectory
figure(2);
for i = 1:length(X)
    plot(