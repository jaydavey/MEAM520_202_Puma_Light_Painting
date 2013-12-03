%generate the target plot
clear all
close all
traj_resolution = 0.1;
plot_resolution = 1; %must be >1

xscale = 2;
yscale = 2;

[X,Y] = meshgrid(-xscale:traj_resolution:xscale, -yscale:traj_resolution:yscale);
Z = -X .* exp(-X.^2 - Y.^2);
%normalize Z so that we can use the height as color data
Z = (Z/max(max(Z))+1)/2*0.8+.1;
zscale = 2;

h1 = figure(1);
mesh(X,Y,Z*zscale);
view_pos = [50,50,40];
axis_val = [-20 20 -20 20 0 zscale];
axis(axis_val);
axis equal
view(view_pos);

%generate an animation of how the robot will follow the trajectory
background_color = [0.3 0.3 0.3];
h2 = figure(2);
set(gcf,'Color',background_color);
set(gca,'Color',background_color);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(view_pos);
axis(axis_val);
axis equal
hold on

grey = [.5 .5 .5];
hue_adjust = 0.85;

%define the origin of the 3D plot in the puma workspace
x_origin = 13; %inches
y_origin = 0;
z_origin = 15; %1 inch of clearance from table top

%create a container to store all of the points we want to paint
painting = zeros(1, 10);

%plot the color of the led at points
counter = 1;
for i = 1:plot_resolution:size(X,1);
    %plot the trajectory
    plot3(X(i,:)+x_origin,Y(i,:)+y_origin,Z(i,:)*zscale+z_origin,'-','color',background_color-.1);
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
            plot3(X(i,j)+x_origin,Y(i,j)+y_origin,Z(i,j)*zscale+z_origin,'.','color',color);
            
            %push the new point into the list of painting points buffer
            painting(counter,1:3) = [X(i,j)+x_origin,Y(i,j)+y_origin,Z(i,j)*zscale+z_origin];
            
            %push the new LED color to the list of painting points buffer
            painting(counter,7:9) = color;
            
            counter = counter+1;
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
            plot3(X(i,j)+x_origin,Y(i,j)+y_origin,Z(i,j)*zscale+z_origin,'.','color',color);
            
            %push the new point into the list of painting points buffer
            painting(counter,1:3) = [X(i,j)+x_origin,Y(i,j)+y_origin,Z(i,j)*zscale+z_origin];
            
            %push the new LED color to the list of painting points buffer
            painting(counter,7:9) = color;
            
            counter = counter+1;
        end
        
    end
    drawnow();
end
for i = 1:plot_resolution:size(Y,1);
    %plot the trajectory
    plot3(X(:,i)+x_origin,Y(:,i)+y_origin,Z(:,i)*zscale+z_origin,'-','color',background_color+.3);
        
    dir = (mod(i,2)*2-1)*-1;
    if dir == 1   
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
            plot3(X(j,i)+x_origin,Y(j,i)+y_origin,Z(j,i)*zscale+z_origin,'.','color',color);
            %push the new point into the list of painting points buffer
            painting(counter,1:3) = [X(j,i)+x_origin,Y(j,i)+y_origin,Z(j,i)*zscale+z_origin];
            
            %push the new LED color to the list of painting points buffer
            painting(counter,7:9) = color;
            
            counter = counter+1;
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
            plot3(X(j,i)+x_origin,Y(j,i)+y_origin,Z(j,i)*zscale+z_origin,'.','color',color);
            painting(counter,1:3) = [X(j,i)+x_origin,Y(j,i)+y_origin,Z(j,i)*zscale+z_origin];
            
            %push the new LED color to the list of painting points buffer
            painting(counter,7:9) = color;
            
            counter = counter+1;
        end

    end
    drawnow();
end

painting(:,5) = painting(:,5)+pi/2;






    