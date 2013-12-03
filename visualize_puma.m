%% Notes
% This file is a heavily modified version of Professor Kuchenbecker's
% scara_robot_kuchenbe.m.  Some deviations that I made from the assignment
% were instead of coloring the entire robot distinct colors when either an
% angle was violated, or the tip went below the table, or another distinct
% color if they were both violated I opted to simply color the link that
% has violated the angle limitation red.  If the robot's end-effector drops
% below the table (z=0) I color the frame 6 axis black.  These decisions
% allow for a more detailed view of what limits are being broken.
function fig_handle = visualize_puma(theta1,theta2,theta3,theta4,theta5,theta6)
    %% SETUP
    %% Define robot parameter variables
    a = 13;
    b = 2.5;
    c = 8;
    d = 2.5;
    e = 8;
    f = 2.5;
    %% Get the angles
    theta_1 = theta1;
    theta_2 = theta2;
    theta_3 = theta3;
    theta_4 = theta4;
    theta_5 = theta5;
    theta_6 = theta6;
    t = [0];

    % Define thresholds
    theta_1_min = -pi;
    theta_1_max = (11/18)*pi;
    theta_2_min = (-5/12)*pi;
    theta_2_max = (4/3)*pi;
    theta_3_min = (-47/36)*pi;
    theta_3_max = (1/3)*pi;
    theta_4_min = (-29/9)*pi;
    theta_4_max = (2/9)*pi;
    theta_5_min = (-2/3)*pi;
    theta_5_max = (11/18)*pi;
    theta_6_min = (-43/36)*pi;
    theta_6_max = (59/36)*pi;
    % Clear the console, so you can more easily find any errors that may occur.
    home

    % Define default link colors
    link1_d = 'b';
    link2_d = 'm';
    link3_d = 'c';
    link4_d = 'g'; %theta 5 corresponds with link 4
    link6_d = 'y';

    % Define default axis colors for frame 6
    x_axis_d = 'r';
    y_axis_d = 'g';
    z_axis_d = 'b';
    %% Step through the data file

    % Initialize a matrix to hold the position of the robot's tip over time.
    % The first row is the x-coordinate, second is y, and third is z, all in
    % the base frame.  You are welcome to make this 4 rows if you want to use
    % the homogeneous representation for points.  It has the same number of
    % columns as t has rows, one tip position for every time step in the
    % simulation.  We keep track of this history so we can trace out the
    % trajectory of where the robot's tip has been.
    tip_history = zeros(3,length(t));

    i=1;
    A1 = [cos(theta_1(i))   -sin(theta_1(i))*cos(pi/2)  sin(theta_1(i))*sin(pi/2)   0;
          sin(theta_1(i))   cos(theta_1(i))*cos(pi/2)   -cos(theta_1(i))*sin(pi/2)  0;
          0                 sin(pi/2)                   cos(pi/2)                   a;
          0                 0                           0                           1];

    A2 = [cos(theta_2(i))	-sin(theta_2(i))*cos(0)     sin(theta_2(i))*sin(0)      c*cos(theta_2(i));
          sin(theta_2(i))	cos(theta_2(i))*cos(0)      -cos(theta_2(i))*sin(0)     c*sin(theta_2(i));
          0                 sin(0)                      cos(0)                      -b;
          0                 0                           0                           1];

    A3 = [cos(theta_3(i))	-sin(theta_3(i))*cos(-pi/2) sin(theta_3(i))*sin(-pi/2)	0;
          sin(theta_3(i))	cos(theta_3(i))*cos(-pi/2)	-cos(theta_3(i))*sin(-pi/2)	0;
          0                 sin(-pi/2)                  cos(-pi/2)                  -d;
          0                 0                           0                           1];

    A4 = [cos(theta_4(i))	-sin(theta_4(i))*cos(pi/2)	sin(theta_4(i))*sin(pi/2)   0;
          sin(theta_4(i))   cos(theta_4(i))*cos(pi/2)   -cos(theta_4(i))*sin(pi/2)  0;
          0                 sin(pi/2)                   cos(pi/2)                   e;
          0                 0                           0                           1];

    A5 = [cos(theta_5(i))	-sin(theta_5(i))*cos(-pi/2)	sin(theta_5(i))*sin(-pi/2)	0;
          sin(theta_5(i))	cos(theta_5(i))*cos(-pi/2)	-cos(theta_5(i))*sin(-pi/2)	0;
          0                 sin(-pi/2)                  cos(-pi/2)                  0;
          0                 0                           0                           1];

    A6 = [cos(theta_6(i))	-sin(theta_6(i))*cos(0)     sin(theta_6(i))*sin(0)      0;
          sin(theta_6(i))	cos(theta_6(i))*cos(0)      -cos(theta_6(i))*sin(0)     0;
          0                 sin(0)                      cos(0)                      f;
          0                 0                           0                           1];

    % Calculate and store T0_6
    T0_6 = A1*A2*A3*A4*A5*A6;  
    T06_history(:,:,i) = T0_6;

    % Create the homogeneous representation of the origin of a frame.
    o = [0 0 0 1]';

    % Calculate the position of the origin of frame 0 expressed in frame 0.
    % By definition, this is just o.
    o0_0 = o;

    % Calculate the position of the origin of frame 1 expressed in frame 0.
    % We multiply A1 into the origin vector to find the position o1.
    o0_1 = A1*o;

    % Calculate the position of the origin of frame 2 expressed in frame 0.
    % We multiply A1 and A2 into the origin vector to find the position o2.
    o0_2 = A1*A2*o;

    % Calculate the position of the origin of frame 3 expressed in frame 0.
    % We multiply A1, A2, and A3 into the origin vector to find the position o3.
    o0_3 = A1*A2*A3*o;

    % Calculate the position of the origin of frame 4 expressed in frame 0.
    % We multiply A1, A2, A3, and A4 into the origin vector to find the position o4.
    o0_4 = A1*A2*A3*A4*o;

    % Calculate the position of the origin of frame 5 expressed in frame 0.
    % We multiply A1, A2, A3, A4, and A5 into the origin vector to find the position o5.
    o0_5 = A1*A2*A3*A4*A5*o;

    % Calculate the position of the origin of frame 6 expressed in frame 0.
    % We multiply A1, A2, A3, A4, A5, and A6 into the origin vector to find the position o6.
    o0_6 = T0_6*o;

    % Put the points together.  Each column is the homogeneous
    % representation of a point in the robot.The points should go in order along the robot.
    % b is the midpoint in the L shape link between joint 1 and 2
    % px,py,pz define the vectors for the axis of the end-effector
    o0_b = A1*[0 0 -2.5 1]';
    u_px = T0_6*[5 0 0 1]';
    u_py = T0_6*[0 5 0 1]';
    u_pz = T0_6*[0 0 5 1]';
    points_to_plot = [o0_0 o0_1 o0_b o0_2 o0_3 o0_4 o0_6];

    % Our list of points to plot is in points_to_plot, going from the base
    % out to the tip.  Each column represents the x, y, and z coordinates
    % of one point.  The last column of points_to_plot should be the
    % position of the tip of the robot.

    % Grab the final plotted point for the trajectory graph.
    tip_history(:,i) = points_to_plot(1:3,end);

    % Check joint angles and set color accordingly.  If an angle goes
    % outside its bounds then the corresponding link turns red. Note that
    % theta 4 and theta 5 both correspond to link 4.
    if theta_1(i) > theta_1_max || theta_1(i) < theta_1_min
        link1 = 'r';
    else
        link1 = link1_d;
    end
    if theta_2(i) > theta_2_max || theta_2(i) < theta_2_min
        link2 = 'r';
    else
        link2 = link2_d;
    end
    if theta_3(i) > theta_3_max || theta_3(i) < theta_3_min
        link3 = 'r';
    else
        link3 = link3_d;
    end
    if theta_4(i) > theta_4_max || theta_4(i) < theta_4_min
        link4 = 'r';
    else
        link4 = link4_d;
    end
    if theta_5(i) > theta_5_max || theta_5(i) < theta_5_min
        link4 = 'r';
    end
    if theta_6(i) > theta_6_max || theta_6(i) < theta_6_min
        link6 = 'r';
    else
        link6 = link6_d;
    end
    % Determine if the robot's tip has gone below z=0. If so set the axis
    % color to all black.
    if o0_6(3) < 0
        x_axis = 'k';
        y_axis = 'k';
        z_axis = 'k';
    else
        x_axis = x_axis_d;
        y_axis = y_axis_d;
        z_axis = z_axis_d;
    end

    % This is a 3D plot with dots at the points and lines connecting
    % neighboring points, made thicker, with big dots, in dark gray.
    hrobot1 = plot3(points_to_plot(1,1:2),points_to_plot(2,1:2),points_to_plot(3,1:2),'.-','linewidth',5,'markersize',20,'color',link1);
    hold on;
    hrobot2 = plot3(points_to_plot(1,2:3),points_to_plot(2,2:3),points_to_plot(3,2:3),'.-','linewidth',5,'markersize',20,'color',link2);
    hold on;
    hrobot3 = plot3(points_to_plot(1,3:4),points_to_plot(2,3:4),points_to_plot(3,3:4),'.-','linewidth',5,'markersize',20,'color',link2);
    hold on;
    hrobot4 = plot3(points_to_plot(1,4:5),points_to_plot(2,4:5),points_to_plot(3,4:5),'.-','linewidth',5,'markersize',20,'color',link3);
    hold on;
    hrobot5 = plot3(points_to_plot(1,5:6),points_to_plot(2,5:6),points_to_plot(3,5:6),'.-','linewidth',5,'markersize',20,'color',link4);
    hold on;
    hrobot6 = plot3(points_to_plot(1,6:7),points_to_plot(2,6:7),points_to_plot(3,6:7),'.-','linewidth',5,'markersize',20,'color',link6);
    hold on;
    % Plot axis of frame 6. Red = x, Green = y, Blue = z.
    hrobot7 = plot3([points_to_plot(1,7),u_px(1)],[points_to_plot(2,7),u_px(2)],[points_to_plot(3,7),u_px(3)],':','linewidth',2,'markersize',20,'color',x_axis);
    hold on;
    hrobot8 = plot3([points_to_plot(1,7),u_py(1)],[points_to_plot(2,7),u_py(2)],[points_to_plot(3,7),u_py(3)],'--','linewidth',2,'markersize',20,'color',y_axis);
    hold on;
    hrobot9 = plot3([points_to_plot(1,7),u_pz(1)],[points_to_plot(2,7),u_pz(2)],[points_to_plot(3,7),u_pz(3)],'-','linewidth',2,'markersize',20,'color',z_axis);
    hold on;

    % Also plot the tip position of the robot, using hold on and hold
    % off, also keeping a handle to the plot so we can update the data
    % points later.
    hold on;
    htip = plot3(tip_history(1,i),tip_history(2,i),tip_history(3,i),'r.');
    hold off;

    % Label the axes.
    xlabel('X (in)');
    ylabel('Y (in)');
    zlabel('Z (in)');

    % Turn on the grid and the box.
    grid on;
    box on;

    % Set the axis limits.
    axis([-15 15 -15 15 -5 25])

    % Set the axis properties for 3D visualization, which makes one
    % unit the same in every direction, and enables rotation.
    axis vis3d;

    % Add a title.
    title('Puma Orientation');
    % Add ability to rotate
    rotate3d on;
    fig_handle = gcf;% Return the figure handle
end