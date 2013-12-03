function thetas = team202_choose_solution(allSolutions, thetasnow)
%% team202_choose_solution.m
%
% Chooses the best inverse kinematics solution from all of the solutions
% passed in.  This decision is based both on the characteristics of the
% PUMA 260 robot and on the robot's current configuration.
%
% This Matlab file provides the starter code for the solution choice
% function of project 2 in MEAM 520 at the University of Pennsylvania.  The
% original was written by Professor Katherine J. Kuchenbecker. Students
% will work in teams modify this code to create their own script. Post
% questions on the class's Piazza forum.
%
% The first input (allSolutions) is a matrix that contains the joint angles
% needed to place the PUMA's end-effector at the desired position and in
% the desired orientation. The first row is theta1, the second row is
% theta2, etc., so it has six rows.  The number of columns is the number of
% inverse kinematics solutions that were found; each column should contain
% a set of joint angles that place the robot's end-effector in the desired
% pose. These joint angles are specified in radians according to the 
% order, zeroing, and sign conventions described in the documentation.  If
% the IK function could not find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.
%
%    allSolutions: IK solutions for all six joints, in radians
%
% The second input is a vector of the PUMA robot's current joint angles
% (thetasnow) in radians.  This information enables this function to
% choose the solution that is closest to the robot's current pose. 
%
%     thetasnow: current values of theta1 through theta6, in radians
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 200.


% You will need to update this function so it chooses intelligently from
% the provided solutions to choose the best one.
%
% There are several reasons why one solution might be better than the
% others, including how close it is to the robot's current configuration
% and whether it violates or obeys the robot's joint limits.
%
% Note that some of the PUMA's joints wrap around, while your solutions
% probably include angles only from -pi to pi or 0 to 2*pi.  If a joint
% wraps around, there can be multiple ways for the robot to achieve the
% same IK solution (the given angle as well as the given angle plus or
% minus 2*pi*n). Be careful about this point.


%% Joint Limits

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

theta_mins = [theta1_min theta2_min theta3_min theta4_min theta5_min theta6_min];
theta_maxs = [theta1_max theta2_max theta3_max theta4_max theta5_max theta6_max];


%% Filter solutions

% Initialize optimal solution distance
opt_sol = [NaN NaN NaN NaN NaN NaN];
opt_sol_dist = NaN;

% Check each solution
for i = 1:size(allSolutions, 2)
    sol = allSolutions(:,i);

    for j = i:length(sol)
        % Check if joint angle violates limit
        angle = sol(j);
        max = theta_maxs(j);
        min = theta_mins(j);
        now = thetasnow(j);
        sol_dist = abs(angle - now);
        if (angle < max && angle > min)
            % Check if wraparound solutions are closer in joint space
            if (abs(sol_dist) > pi)
                add = angle + 2*pi;
                if (abs(add - now) < sol_dist && add < max && add > min)
                    sol(j) = add;
                    sol_dist = abs(add - now);
                end

                sub = angle - 2*pi;
                if (abs(sub - now) < sol_dist && sub < max && sub > min)
                    sol(j) = sub;
                    sol_dist = abs(sub - now);
                end
            end
            
            % Check if the solution is closer than the previous optimal (or
            % the first workable solution found)
            sol_dist = norm(sol - thetasnow);
            if (sol_dist < opt_sol_dist || isnan(opt_sol_dist))
                opt_sol = sol;
                opt_sol_dist = sol_dist;
            end
        
        % Check if wraparound solutions violate joint angle limits
        else
            sol_found = false;
            add = angle + 2*pi;
            if (abs(add - now) < sol_dist && add < max && add > min)
                sol(j) = add;
                sol_dist = abs(add - now);
                sol_found = true;
            end

            sub = angle - 2*pi;
            if (abs(sub - now) < sol_dist && sub < max && sub > min)
                sol(j) = sub;
                sol_dist = abs(sub - now);
                sol_found = true;
            end
            
            if (sol_found)
                % Check if the solution is closer than the previous optimal
                % (or the first workable solution found)
                sol_dist = norm(sol - thetasnow);
                if (sol_dist < opt_sol_dist || isnan(opt_sol_dist))
                    opt_sol = sol;
                    opt_sol_dist = sol_dist;
                end
            end
        end
    end
end

thetas = opt_sol;
        
    