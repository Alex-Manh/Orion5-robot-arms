%% Clear the workspace
clear all; close all; clc;

%% Robot Arm Part - no vision

% Initial Coordinates
initial_destination = [55, 168, 268;  % - x coord initial
                       420, 391, 370]; % - y coord initial
% Final Coordinates
final_destination = [83, 183, 290; % - x coord final
                     65, 97, 136]; % - y coord final

%% Start the communication with orion
claw = TheClaw();
for id = claw.BASE:claw.WRIST
    claw.setJointTorqueEnable(id, 1); % Enable torque on motor
    claw.setJointControlMode(id, claw.POS_TIME); % Set mode to time to position
    claw.setJointTimeToPosition(id, 2); % Set joint time to position to 2 seconds
end

%% Move Robot to pickup location


starting_joint_angles = [0, 90, 90, 90, 300];
claw.setAllJointsPosition(starting_joint_angles);
pause(2);

%Position 1
pick(claw, initial_destination(1,1), initial_destination(2,1));
drop(claw, final_destination(1,1), final_destination(2,1))
%claw.setAllJointsPosition(starting_joint_angles);

%Position 2
pick(claw, initial_destination(1,2), initial_destination(2,2));
drop(claw, final_destination(1,2), final_destination(2,2))
%claw.setAllJointsPosition(starting_joint_angles);

%Position 3
pick(claw, initial_destination(1,3), initial_destination(2,3));
drop(claw, final_destination(1,3), final_destination(2,3))
claw.setAllJointsPosition(starting_joint_angles);

%% Reverse 

starting_joint_angles = [0, 90, 90, 90, 300];
claw.setAllJointsPosition(starting_joint_angles);
pause(2);

%Position 1
pick(claw, final_destination(1,1), final_destination(2,1))
drop(claw, initial_destination(1,1), initial_destination(2,1));

%claw.setAllJointsPosition(starting_joint_angles);

%Position 2
pick(claw, final_destination(1,2), final_destination(2,2))
drop(claw, initial_destination(1,2), initial_destination(2,2));

%claw.setAllJointsPosition(starting_joint_angles);

%Position 3
pick(claw, final_destination(1,3), final_destination(2,3))
drop(claw, initial_destination(1,3), initial_destination(2,3));

claw.setAllJointsPosition(starting_joint_angles);

%% Stop the arm
claw.stop();

%% Defined functions for arm movement
%Pick, Drop
function drop(claw, x, y)
    %Get there
    angle1 = pt2angle(x, y, 70);
    move_to(claw, angle1);
    %Bow
    pause(2);
    angle2 = pt2angle(x, y, 12);
    move_to(claw, angle2);
    %Drop
    pause(2);
    open_claw(claw);
    %Up
    pause(2);
    move_to(claw, angle1); 
    pause(2);

end

function pick(claw, x, y)
    %open hand
    open_claw(claw);
    
    %Get there
    angle1 = pt2angle(x, y, 70);
    move_to(claw, angle1);

    %Bow
    pause(3);
    angle2 = pt2angle(x, y, 10);
    move_to(claw, angle2);
    pause(2);

    %Grab it
    close_claw(claw);
    pause(2.5);
    
    %Stand up
    move_to(claw, angle1);
    pause(2);
end

function move_to(claw, angles)
    %Claw.CLAW will be unchanged
    ANGLES = claw.getAllJointsPosition();
    claw_position = ANGLES(5);
    %claw_open = 300;
    
    joint_angles = [angles(1)-4, angles(2), angles(3), angles(4)+3, claw_position];
    claw.setAllJointsPosition(joint_angles);
end

%Claw movement
function close_claw(claw)
    claw.setJointPosition(claw.CLAW, 170);
end

function open_claw(claw)
    claw.setJointPosition(claw.CLAW, 300);
end

%Read and calculate needed angles
function angles = pt2angle(x,y,z)
    %Arm length
    h = 53;
    r = 30.309;
    L2 = 170.384;
    L3 = 136.307;
    L4 = 86 + 40;
    
    % Base offset
    x = x-100;
    y = 290-y; 
    
    %Calculation
    alpha = deg2rad(90);
    g = sqrt(x^2 + y^2);
    i = cos(alpha)*L4;
    j = sin(alpha)*L4;

    i2 = g - r - i;
    j2 = z - h + j;
    r2 = sqrt(i2^2 + j2^2);

    A = (r2^2 - L2^2 - L3^2)/(-2 * L2 * L3);
    theta3 = atan2_mine(A, sqrt(1-A^2));

    B = (j2^2 - r2^2 - i2^2)/(-2 * r2 * i2);
    beta1 = atan2_mine(B, sqrt(1-B^2));
    C = (L3^2 - r2^2 - L2^2)/(-2 * L2 * r2);
    beta2 = atan2_mine(C, sqrt(1-C^2));

    theta2 = beta1 + beta2;

    theta1 = atan2(y,x);

    theta4 = deg2rad(270) - theta3 - theta2;
    
    theta1 = rad2deg(theta1);
    %fprintf('Theta 1: %.0f\n',theta1);
    theta2 = rad2deg(theta2);
    %fprintf('Theta 2: %.0f\n',theta2);
    theta3 = rad2deg(theta3);
    %fprintf('Theta 3: %.0f\n',theta3);
    theta4 = rad2deg(theta4);
    %fprintf('Theta 4: %.0f\n',theta4);
    
    angles = [theta1, theta2, theta3, theta4];
end

%Atan 
function output = atan2_mine(x, y)
    if x > 0.0
        output = atan(y/x);
        return;
    end
    if x < 0.0
        if y >= 0.0
            output = pi + atan(y/x);
            return;
        else
            output = -pi + atan(y/x);
            return ;
        end
    end
    
    if y > 0.0 % x == 0
        output = pi/2;
        return;
    end
    if y < 0.0
        output = -pi/2;
        return;
    end
    
    output = 0.0; % Should be undefined
    return;
end

