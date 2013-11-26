function thetas = team202_puma_ik(x, y, z, phi, theta, psi)
%% team202_puma_ik.m
%
% Calculates the full inverse kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 inverse
% kinematics function of project 2 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker. Students will work in teams modify this code to create
% their own script. Post questions on the class's Piazza forum.
%
% The first three input arguments (x, y, z) are the desired coordinates of
% the PUMA's end-effector tip in inches, specified in the base frame.  The
% origin of the base frame is where the first joint axis (waist) intersects
% the table. The z0 axis points up, and the x0 axis points out away from
% the robot, perpendicular to the front edge of the table.  These arguments
% are mandatory.
%
%     x: x-coordinate of the origin of frame 6 in frame 0, in inches
%     y: y-coordinate of the origin of frame 6 in frame 0, in inches
%     z: z-coordinate of the origin of frame 6 in frame 0, in inches
%
% The fourth through sixth input arguments (phi, theta, psi) represent the
% desired orientation of the PUMA's end-effector in the base frame using
% ZYZ Euler angles in radians.  These arguments are mandatory.
%
%     phi: first ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     theta: second ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     psi: third ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%
% The output (thetas) is a matrix that contains the joint angles needed to
% place the PUMA's end-effector at the desired position and in the desired
% orientation. The first row is theta1, the second row is theta2, etc., so
% it has six rows.  The number of columns is the number of inverse
% kinematics solutions that were found; each column should contain a set
% of joint angles that place the robot's end-effector in the desired pose.
% These joint angles are specified in radians according to the
% order, zeroing, and sign conventions described in the documentation.  If
% this function cannot find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 200.


%% CHECK INPUTS

% Look at the number of arguments the user has passed in to make sure this
% function is being called correctly.
if (nargin < 6)
    error('Not enough input arguments.  You need six.')
elseif (nargin == 6)
    % This the correct way to call this function, so we don't need to do
    % anything special.
elseif (nargin > 6)
    error('Too many input arguments.  You need six.')
end


%% ROBOT DIMENSIONS

% Define the robot's measurements
a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches

%% Robot Parameters

% Define joint limits
theta1_min = degtorad(-180);
theta1_max = degtorad(110);
theta2_min = degtorad(-75);
theta2_max = degtorad(240);
theta3_min = degtorad(-235);
theta3_max = degtorad(60);
theta4_min = degtorad(-580);
theta4_max = degtorad(40);
theta5_min = degtorad(-120);
theta5_max = degtorad(110);
theta6_min = degtorad(-215);
theta6_max = degtorad(295);

theta_mins = [theta1_min, theta2_min, theta3_min, theta4_min, theta5_min, theta6_min];
theta_maxs = [theta1_max, theta2_max, theta3_max, theta4_max, theta5_max, theta6_max];



%% CALCULATE INVERSE KINEMATICS SOLUTION(S)

% For now, just set the first solution to NaN (not a number) and the second
% to zero radians.  You will need to update this code.
% NaN is what you should output if there is no solution to the inverse
% kinematics problem for the position and orientation that were passed in.
% For example, this would be the correct output if the desired position for
% the end-effector was outside the robot's reachable workspace.  We use
% this sentinel value of NaN to be sure that the code calling this function
% can tell that something is wrong and shut down the PUMA.

% Calculate rotation matrix to define orientation of end effector.
R06 = [cos(phi)*cos(theta)*cos(psi) - sin(phi)*sin(psi)  -cos(phi)*cos(theta)*sin(psi) - sin(phi)*cos(psi)  cos(phi)*sin(theta);
       sin(phi)*cos(theta)*cos(psi) + cos(phi)*sin(psi)  -sin(phi)*cos(theta)*sin(psi) + cos(phi)*cos(psi)  sin(phi)*sin(theta);
       -sin(theta)*cos(psi)                              sin(theta)*sin(psi)                                cos(theta)];

% Calculate wrist center
xc = x - f*R06(1, 3);
yc = y - f*R06(2, 3);
zc = z - f*R06(3, 3);

% Calculate the first three joint angles.  The first joint variable th1_1
% corresponds to the second and third joint variable pairings (th2_1
% th_3_1) and (th2_2 and th3_2), the first joint variable th1_2 corresponds
% to the second and third joint variable pairings (th2_3 th3_3) and (th2_4
% th3_4).

%solve for angles and catch any impossible solutions
try

    % Calculate theta1
    th1_1 = atan2(yc, xc) - atan2((b + d), sqrt(xc^2 + yc^2 - (b+d)^2));
    th1_2 = atan2(yc, xc) + atan2(-(b + d), -sqrt(xc^2 + yc^2 - (b+d)^2));

    % Calculate theta3
    D = ((xc^2 + yc^2 - (b + d)^2 + (zc - a)^2 - c^2 - e^2)/(2*c*e));

 
    th3_1 = atan2(sqrt(1-D^2), D) - pi/2;
    th3_2 = atan2(-sqrt(1-D^2), D) - pi/2;
    th3_3 = pi - th3_1;
    th3_4 = pi - th3_2;
    % Calculate theta2
    th2_1 = atan2((zc - a), (sqrt(xc^2 + yc^2 - (b + d)^2))) - atan2((e*cos(th3_1)), (c - e*sin(th3_1)));
    th2_2 = atan2((zc - a), (sqrt(xc^2 + yc^2 - (b + d)^2))) - atan2((e*cos(th3_2)), (c - e*sin(th3_2)));
    th2_3 = pi - th2_1;
    th2_4 = pi - th2_2;
    
    % Create variables containing four position configurations of arm
    position_angles = [th1_1 th1_1 th1_2 th1_2;
                       th2_1 th2_2 th2_3 th2_4;
                       th3_1 th3_2 th3_3 th3_4];

    % Initializse a variable in which we can store Euler Angles               
    n = size(position_angles,2);
    Euler_angles = zeros(3, 2*n);

    for i = 1:n
        theta1 = position_angles(1,i);
        theta2 = position_angles(2,i);
        theta3 = position_angles(3,i);
        A1 = dh_kuchenbe(0,  pi/2,   a, theta1);
        A2 = dh_kuchenbe(c,     0,  -b, theta2);
        A3 = dh_kuchenbe(0, -pi/2,  -d, theta3);
        T03 = A1*A2*A3;
        R03 = T03(1:3,1:3);
        R36 = R03'*R06;

        % Solve for Euler angles (and catch wrist singularity?)
        
        Euler_angles(2,i) = -atan2(sqrt(1 - R36(3, 3)^2), R36(3, 3));
        Euler_angles(2,i+n) = -atan2(-sqrt(1 - R36(3, 3)^2), R36(3, 3));

        Euler_angles(1,i) = atan2(R36(2, 3), R36(1, 3));
        Euler_angles(1,i+n) = atan2(-R36(2, 3), -R36(1, 3));

        Euler_angles(3,i) = atan2(R36(3, 2), -R36(3, 1));
        Euler_angles(3,i+n) = atan2(-R36(3, 2), R36(3, 1));
    end

    final_thetas = [position_angles position_angles; Euler_angles];
    th1 = final_thetas(1,:);
    th2 = final_thetas(2,:);
    th3 = final_thetas(3,:);
    th4 = final_thetas(4,:);
    th5 = final_thetas(5,:);
    th6 = final_thetas(6,:);


    for j = 1:size(final_thetas,2)
        [points_to_plot x06 y06 z06] = puma_fk_kuchenbe(th1(j),th2(j),th3(j),th4(j),th5(j),th6(j));
        o6 = points_to_plot(1:3,8);
        if (abs(x) < (abs(o6(1))+.01)) && (abs(y) < (abs(o6(2))+.01)) && (abs(z) < (abs(o6(3))+.01))
    %         disp('Your IK works!')
        else
            fk = points_to_plot(1:3,8)'
            ik = [x y z]
            error('Something in your inverse kinematics does not match your foward kinematics!')
        end
    end
    
catch
    

    %checking for wrist center solution
    thetas = [NaN;NaN;NaN;NaN;NaN;NaN];
    fprintf('No solution to inverse kinematics.  Target outside dexterous workspace!\n')

    return
    
end

%% FORMAT OUTPUT

% Put all of the thetas into a column vector to return.
thetas = [th1; th2; th3; th4; th5; th6];

% By the very end, each column of thetas should hold a set of joint angles
% in radians that will put the PUMA's end-effector in the desired
% configuration.  If the desired configuration is not reachable, set all of
% the joint angles to NaN.