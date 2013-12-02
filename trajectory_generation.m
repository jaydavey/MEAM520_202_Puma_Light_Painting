%generate the target plot
clear all
close all
traj_resolution = 0.1;
plot_resolution = 1; %must be >1

[X,Y] = meshgrid(-2:traj_resolution:2, -2:traj_resolution:2);
Z = X .* exp(-X.^2 - Y.^2);
%normalize Z so that we can use the height as color data
Z = (Z/max(max(Z))+1)/2*0.8+.1;
zscale = 4;

h1 = figure(1);
mesh(X,Y,Z*zscale);
view_pos = [-5,-5,2];
axis_val = [-2 2 -2 2 0 zscale];
axis(axis_val);
axis equal
view(view_pos);

%generate an animation of how the robot will follow the trajectory
h2 = figure(2);
set(gcf,'Color',[0.1 0.1 0.1]);
set(gca,'Color',[0.1 0.1 0.1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(view_pos);
axis(axis_val);
axis equal
hold on

grey = [.5 .5 .5];
hue_adjust = 0.85;

%plot the color of the led at points
for i = 1:size(X,1);
    %plot the trajectory
    plot3(X(i,:),Y(i,:),Z(i,:)*zscale,'-','color',grey-.1);
    dir = (mod(i,2)*2-1)*-1;
    if dir == 1    
        %plot the led color
        for j = 1:plot_resolution:size(X,2);
            %adjust the hue to make green the 'zero' value
            z = 1-Z(i,j);
            if z+hue_adjust>1
                z = z+hue_adjust-1;
            else
                z = z+hue_adjust;
            end
            %obtain a color based on height and hue adjustment
            color = hsv2rgb([z 1 1]);
            %plot the result
            plot3(X(i,j),Y(i,j),Z(i,j)*zscale,'.','color',color);
        end
    else
        %plot the led color
        for j = size(X,2):-plot_resolution:1;
            %adjust the hue to make green the 'zero' value
            z = 1-Z(i,j);
            if z+hue_adjust>1
                z = z+hue_adjust-1;
            else
                z = z+hue_adjust;
            end
            %obtain a color based on height and hue adjustment
            color = hsv2rgb([z 1 1]);
            %plot the result
            plot3(X(i,j),Y(i,j),Z(i,j)*zscale,'.','color',color);
        end
        
    end
    drawnow();
end
for i = 1:size(Y,1);
    dir = (mod(i,2)*2-1)*-1;
    if dir == 1   
        %plot the trajectory
        plot3(X(:,i),Y(:,i),Z(:,i)*zscale,'-','color',grey+.3);
        %plot the led color
        for j = 1:plot_resolution:size(Y,2);
            %adjust the hue to make green the 'zero' value
            z = 1-Z(j,i);
            if z+hue_adjust>1
                z = z+hue_adjust-1;
            else
                z = z+hue_adjust;
            end
            %obtain a color based on height and hue adjustment
            color = hsv2rgb([z 1 1]);
            %plot the result
            plot3(X(j,i),Y(j,i),Z(j,i)*zscale,'.','color',color);
        end
    else
        for j = size(X,2):-plot_resolution:1;            
            %adjust the hue to make green the 'zero' value
            z = 1-Z(j,i);
            if z+hue_adjust>1
                z = z+hue_adjust-1;
            else
                z = z+hue_adjust;
            end
            %obtain a color based on height and hue adjustment
            color = hsv2rgb([z 1 1]);
            %plot the result
            plot3(X(j,i),Y(j,i),Z(j,i)*zscale,'.','color',color);
        end

    end
    drawnow();
end





    