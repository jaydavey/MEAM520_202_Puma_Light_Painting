%% team202_testing.m
% 
% This Matlab script is part of the starter code for the inverse
% kinematics part of Project 2 in MEAM 520 at the University of Pennsylvania.
% It tests the team's inverse kinematics code for various configurations.
% The final version was written by team 202 (Alex McCraw, Jay Davey, and 
% Vivienne Clayton)

%% SETUP

% Clear all variables from the workspace.
clear all

% Clear the console, so you can more easily find any errors that may occur.
clc

% Set whether to animate the robot's movement and how much to slow it down.
pause on;  % Set this to off if you don't want to watch the animation.
GraphingTimeDelay = 1;%0.05; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.


%% ROBOT DIMENSIONS
a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches


%% CHOOSE INPUT PARAMETERS

% Set the test type.
testType = 1;

switch(testType)

    case 1

        % One position and orientation that you can manually modify.
        
        % Define the configuration.
        ox_history = c + e + f; % inches
        oy_history = b + d; % inches
        oz_history = a; % inches
        phi_history = 0; % radians
        theta_history = pi/2; % radians
        psi_history = 0; % radians
        
    case 2
        
        % Linear interpolation between two points and two sets of Euler angles.
        
        % Create a time vector.
        t = (0:0.2:2*pi)';

        % Define starting configuration.
        ox_start = 10.5; % inches
        oy_start = -5; % inches
        oz_start = 21; % inches
        phi_start = 0; % radians
        theta_start = -pi/2; % radians
        psi_start = 0; % radians

        % Define ending configuration.
        ox_end = 10.5; % inches
        oy_end = -5; % inches
        oz_end = 21; % inches
        phi_end = 0; % radians
        theta_end = pi/2; % radians
        psi_end = 0; % radians

        % Do the interpolation.
        ox_history = ox_start * ones(length(t),1) + (ox_end - ox_start)*(t - t(1))./(t(end) - t(1));
        oy_history = oy_start * ones(length(t),1) + (oy_end - oy_start)*(t - t(1))./(t(end) - t(1));
        oz_history = oz_start * ones(length(t),1) + (oz_end - oz_start)*(t - t(1))./(t(end) - t(1));
        phi_history = phi_start * ones(length(t),1) + (phi_end - phi_start)*(t - t(1))./(t(end) - t(1));
        theta_history = theta_start * ones(length(t),1) + (theta_end - theta_start)*(t - t(1))./(t(end) - t(1));
        psi_history = psi_start * ones(length(t),1) + (psi_end - psi_start)*(t - t(1))./(t(end) - t(1));
        
    case 3

        % A vertical circle parallel to the x-z plane.

        % Define the radius of the circle.
        radius = 4; % inches
        
        % Define the y-value for the plane that contains the circle.
        y_offset = 7; % inches
        
        % Define the x and z coordinates for the center of the circle.
        x_center = 4; % inches
        z_center = 15; % inches
        
        % Create a time vector.
        t = (0:0.2:2*pi)';
        
        % Set the desired x, y, and z positions over time given the circle parameters.
        ox_history = x_center + radius * sin(t);
        oy_history = y_offset * ones(size(t));
        oz_history = z_center + radius * cos(t);
        
        % Set the desired Euler Angles to be constant for the duration of
        % the test.
        phi_history = 0*ones(length(t),1); % radians
        theta_history = 0*ones(length(t),1); % radians
        psi_history = pi/4*ones(length(t),1); % radians
        
    case 4
        
        % A set number of random positions and random orientations.  
        
        % Note that these are not all guaranteed to be reachable, nor
        % is this function guaranteed to cover the full reachable
        % workspace.  But it's a decent start.
        
        % Set the number of random poses to generate.
        nPoses = 10;
        
        % Pick a random radius approximately matched to the size of the robot.
        rr = 5 + rand(nPoses,1) * (10); % inches
        
        % Pick random values for two angles of spherical coordinates.
        thr = rand(nPoses,1) * 2 * pi; % radians
        phr = -pi/3 + rand(nPoses,1) * 2*pi/3; % radians
        
        % Set random tip position.
        ox_history = rr .* cos(phr) .* cos(thr);
        oy_history = rr .* cos(phr) .* sin(thr);
        oz_history = 13 + rr .* sin(phr);
        
        % Set random Euler angles.
        phi_history = rand(nPoses,1) * 2 * pi;
        theta_history = -pi/2 + rand(nPoses,1) * pi;
        psi_history = rand(nPoses,1) * 2 * pi;
        
    case 5
        
        % Run random joint angles through the forward kinematic and use the
        % resultant position and orientation to test the inverse kinematic,
        % checking the result against the original.  Run a large number of
        % tests and return the mean and max deviations found for position
        % and orientation.  Plotting does not occur during this test case
        % to allow it to finish faster.
        
        % Set the number of test runs
        n = 10000;
        
        % Initialize matrices to hold deviation
        pos_dev = zeros(8*n, 1);
        rot_dev = zeros(8*n, 1);
        
        for i = 1:n
            % Generate random joint angles
            th_rand = rand(6, 1)*2*pi;
            
            % Run joint angles through FK
            [points, x06, y06, z06] = puma_fk_kuchenbe(th_rand(1), th_rand(2), th_rand(3), th_rand(4), th_rand(5), th_rand(6));
            
            % Pull out position and orientation given by FK
            o = points(1:3, 8);
            ux = (x06(1:3, 2) - x06(1:3, 1)); ux = ux/norm(ux);
            uy = (y06(1:3, 2) - y06(1:3, 1)); uy = uy/norm(uy);
            uz = (z06(1:3, 2) - z06(1:3, 1)); uz = uz/norm(uz);
            R = [ux uy uz];
            
            % Solve for Euler angles if at first singularity
            if (R(3, 3) == 1)
                theta = 0;
                phi = 0;
                psi = atan2(R(2, 1), R(1, 1));

            % Solve for Euler angles if at second singularity
            elseif (R(3, 3) == -1)
                theta = pi;
                phi = 0;
                psi = -atan2(-R(1, 2), -R(1, 1));

            % Solve for Euler angles if not at singularity
            else
                theta = atan2(sqrt(1 - R(3, 3)^2), R(3, 3));
                phi = atan2(R(2, 3), R(1, 3));
                psi = atan2(R(3, 2), -R(3, 1));
            end
            
            % Run the IK using the previously found position and
            % orientation
            th_IK = team202_puma_ik(o(1), o(2), o(3), phi, theta, psi);
            
            % Run the FK using the joint angles given by the IK for each
            % configuration
            for j = 1:8
                th_check = th_IK(1:6, j);
                [points_check, x06_check, y06_check, z06_check] = puma_fk_kuchenbe(th_check(1), th_check(2), th_check(3), th_check(4), th_check(5), th_check(6));
                o_check = points_check(1:3, 8);

                % Calculate positional deviation
                pos_dev(i + (j - 1)*n) = norm(o_check - o);

                % Calculate orientation devistion
                ux_check = (x06_check(1:3, 2) - x06_check(1:3, 1)); ux_check = ux_check/norm(ux_check);
                uy_check = (y06_check(1:3, 2) - y06_check(1:3, 1)); uy_check = uy_check/norm(uy_check);
                uz_check = (z06_check(1:3, 2) - z06_check(1:3, 1)); uz_check = uz_check/norm(uz_check);
                ux_dev = norm(ux_check - ux);
                uy_dev = norm(uy_check - uy);
                uz_dev = norm(uz_check - uz);
                rot_dev(i + (j - 1)*n) = norm([ux_dev, uy_dev, uz_dev]);
            end
            
        end
        
        disp(['The mean positional deviation is: ', num2str(mean(pos_dev)), ' in'])
        disp(['The max positional deviation is: ', num2str(max(pos_dev)), ' in'])
        disp(['The mean rotational deviation is: ', num2str(mean(rot_dev))])
        disp(['The max rotational deviation is: ', num2str(max(rot_dev))])
        
        break
    
    case 6
        % Enter thetas, push through FK. Use FK output as input to IK.
        
        th1 = 0;
        th2 = 0;
        th3 = 0;
        th4 = 0;
        th5 = 0;
        th6 = 0;
        
       [points_to_plot, x06, y06, z06] = puma_fk_kuchenbe(th1,th2,th3,th4,th5,th6);
       
       % For tip position, simply use the o6 position! 
       o6 = points_to_plot(1:3,8);
       
       ox_history = o6(1);
       oy_history = o6(2);
       oz_history = o6(3);
      
       ux = (x06(1:3,2) - x06(1:3,1)); ux = ux / norm(ux);
       uy = (y06(1:3,2) - y06(1:3,1)); uy = uy / norm(uy);
       uz = (z06(1:3,2) - z06(1:3,1)); uz = uz / norm(uz);
       
       R06 = [ux uy uz];
       
       theta_history = -atan2(sqrt(1 - R06(3, 3)^2), R06(3, 3));
%        theta_history = -atan2(-sqrt(1 - R06(3, 3)^2), R06(3, 3));

        phi_history = atan2(R06(2, 3), R06(1, 3));
%         phi_history = atan2(-R06(2, 3), -R06(1, 3));

        psi_history = atan2(R06(3, 2), -R06(3, 1));
%         psi_history = atan2(-R06(3, 2), R06(3, 1));

    otherwise
        error(['Unknown testType: ' num2str(testType)])
end    
    

%% TEST

% Notify the user that we're starting the test.
disp('Starting the test.')

% Show a message to explain how to cancel the test and graphing.
disp('Click in this window and press control-c to stop the code.')

% Plot the robot once in the home position to get the plot handles.
figure(1)
h = plot_puma_kuchenbe(0,0,0,0,0,0,0,0,0,0,-pi/2,0,0,0,0);
%view([0,0,100]);

% Step through the ox_history vector to test the inverse kinematics.
for i = 1:length(ox_history)
    
    % Pull the current values of ox, oy, and oz from their histories. 
    ox = ox_history(i);
    oy = oy_history(i);
    oz = oz_history(i);
    
    % Pull the current values of phi, theta, and psi from their histories. 
    phi = phi_history(i);
    theta = theta_history(i);
    psi = psi_history(i);

    % Send the desired pose into the inverse kinematics to obtain the joint
    % angles that will place the PUMA's end-effector at this position and
    % orientation relative to frame 0.
    thetas = team202_puma_ik(ox, oy, oz, phi, theta, psi);
    
    % For each of the columns in thetas.
    s = size(thetas);
    for j = 1:s(2)

        % Plot the robot at this IK solution.        
        plot_puma_kuchenbe(ox,oy,oz,phi,theta,psi,thetas(1,j),thetas(2,j),thetas(3,j),thetas(4,j),thetas(5,j),thetas(6,j),0,1,0,h);

        % Set the title to show the viewer which pose and result this is.
        title(['Test ' num2str(testType) ' - Pose ' num2str(i) ' - Solution ' num2str(j)])
        
        % Pause for a short duration so that the viewer can watch animation evolve over time.
        pause(GraphingTimeDelay)

    end
    
    % Pause for a short duration so that the viewer can watch animation evolve over time.
    pause(GraphingTimeDelay)
    
end

%% FINISHED

% Tell the use.
disp('Done with the test.')
