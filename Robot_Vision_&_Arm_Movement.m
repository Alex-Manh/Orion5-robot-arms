%% Clear the workspace
clear all; close all; clc;

%% Visual Part

%% Load the images into the script
im_initial = imread('initial.jpeg');
im_initial = imrotate(im_initial,-90,'bilinear');

im_destination = imread('destination.jpeg');
im_destination = imrotate(im_destination ,-90,'bilinear');

im_worksheet = imread('worksheet.jpeg');
im_worksheet = imrotate(im_worksheet ,-90,'bilinear');
%%
% Extract Blob information from images
[im_initial_blobs, im_initial_colour] = threshold_blob_image(im_initial, 0);
[im_destination_blobs, im_destination_colour] = threshold_blob_image(im_destination, 0);

[im_worksheet_blobs, im_worksheet_colour, calibration_blobs] = threshold_blob_image(im_worksheet, 1);

% Classify Blob information from images
% Follows this format: 
%Shape Type (0=square;1=triangle;2=circle), Uv, Area, Size(1=large), Colour(1=red)
initial_data = classify_blobs(im_initial_blobs, im_initial_colour, 0);
destination_data = classify_blobs(im_destination_blobs, im_destination_colour, 0);

worksheet_data = classify_blobs(im_worksheet_blobs, im_worksheet_colour, 1);

% Find initial and destination blobs in the worksheet image

index_initial = find_blobs(worksheet_data, initial_data);
index_destination = find_blobs(worksheet_data, destination_data);

% Calculate the homography of the found blobs in the worksheet
% Get the size of the worksheet
[image_size_u image_size_v] = size(im_worksheet);

H = calculate_homography_worksheet(image_size_u, image_size_v, calibration_blobs);

initial_homography_points = calculate_homography_points(H, im_worksheet_blobs, index_initial);
destination_homography_points = calculate_homography_points(H, im_worksheet_blobs, index_destination);

% Plot the required shapes

%Yellow centre for initial
%Green centre for dest
for i=1:length(index_initial)
    im_worksheet_blobs(index_initial(i)).plot('y*');
end
for i=1:length(index_destination)
    im_worksheet_blobs(index_destination(i)).plot('g*');
end

% Print out 
clc;
fprintf('Initial Position\n\n');
fprintf('Shape  \t Colour\t Size\t Blob Position\n');
fprintf('-----------------------------------------\n');
print_data(initial_data, initial_homography_points);

fprintf('\n\n');

fprintf('Destination Position\n\n');
fprintf('Shape\t Colour\t Size\t Blob Position\n');
fprintf('-----------------------------------------\n');
print_data(destination_data, destination_homography_points);

%% Robot Arm Part

% Start the communication with orion
claw = TheClaw();
for id = claw.BASE:claw.WRIST
    claw.setJointTorqueEnable(id, 1); % Enable torque on motor
    claw.setJointControlMode(id, claw.POS_TIME); % Set mode to time to position
    claw.setJointTimeToPosition(id, 2); % Set joint time to position to 2 seconds
end

% Move Robot to pickup location
starting_joint_angles = [0, 90, 90, 90, 300];
claw.setAllJointsPosition(starting_joint_angles);
pause(2);

%Position 1
pick(claw, initial_homography_points(1,1), initial_homography_points(2,1));
drop(claw, destination_homography_points(1,1), destination_homography_points(2,1))
%claw.setAllJointsPosition(starting_joint_angles);

%Position 2
pick(claw, initial_homography_points(1,2), initial_homography_points(2,2));
drop(claw, destination_homography_points(1,2), destination_homography_points(2,2))
%claw.setAllJointsPosition(starting_joint_angles);

%Position 3
pick(claw, initial_homography_points(1,3), initial_homography_points(2,3));
drop(claw, destination_homography_points(1,3), destination_homography_points(2,3))
claw.setAllJointsPosition(starting_joint_angles);
%%
% Stop the arm
claw.stop();

%% Function Part

%% Defined functions used for visual
function [output_blobs,colour,calibration_blobs] = threshold_blob_image(image, worksheet)
    % Break the image down into colour planes
    im_red = image(:,:,1);
    im_green = image(:,:,2);
    im_blue = image(:,:,3);

    %Normalise the image's
    im_red = double(im_red) /255;
    im_green = double(im_green) /255;
    im_blue = double(im_blue) /255;

    %Gamma Correct the images
    im_red = im_red.^2.5;
    im_green = im_green.^2.5;
    im_blue = im_blue.^2.0; %TAKE BETTER PHOTOS AND SET THIS BACK TO 2.5

    %Calculate the colours chromaticity
    im_red = im_red./(im_blue + im_green + im_red);
    im_green = im_green./(im_blue + im_green + im_red);
    im_blue = im_blue./(im_blue + im_green + im_red);

    % Threshold out the colours
    im_red_things = im_red>0.5;
    im_green_things = im_green>0.5;
    im_blue_things = im_blue>0.485;
   

    %Close the gaps in the image
    im_red_things = imopen(im_red_things, strel('disk', 12));
    im_green_things = imopen(im_green_things, strel('disk', 12));
    %im_blue_things = imclose(im_blue_things, strel('disk', 16));

    im_blue_things = imopen(im_blue_things, strel('disk', 12));
    idisp(im_blue_things)
    %Create blob objects from the thresholded images
    %Create 1 red to compare colours with
    imr = iblobs(im_red_things, 'area', [10000, 4000000], 'boundary');
    imrg = iblobs(im_red_things+im_green_things, 'area', [10000, 4000000], 'boundary');
  
    %figure(2);
    %idisp(im_red_things+im_green_things);
    colour = zeros(1,length(imrg));

    %Compare areas from the red and green image to the red
    for i=1:length(imrg)
        for j=1:length(imr)
            if imrg(i).area == imr(j).area% If Area matches with one of the shapes it must be green
                colour(i) = 1;
                break;
            else
                colour(i) = 0;
            end
        end
    end
    %output is the blobs object of both the red and green images
    output_blobs = imrg;

    if worksheet == 1
        calibration_blobs = iblobs(im_blue_things, 'area', [5000, 300000]);
    end
end

function output = calculate_homography_points(H, blobs, index)

    for i=1:length(index)
        p = [blobs(index(i)).uc blobs(index(i)).vc];
        q = homtrans(H, p')
        output(1,i) = q(1);
        output(2,i) = q(2);
    end


end

function H = calculate_homography_worksheet(image_size_u, image_size_v, calibration_blobs)

    % Get the first coloumn of points
    label = find([calibration_blobs.uc] < image_size_u/2+image_size_u/3 & [calibration_blobs.uc] > image_size_u/3);
    % Get the first point - top left
    Pb(1, 1) = calibration_blobs(label(1)).uc;
    Pb(2, 1) = calibration_blobs(label(1)).vc;
    % Get the second point - bottom left
    Pb(1, 2) = calibration_blobs(label(3)).uc;
    Pb(2, 2) = calibration_blobs(label(3)).vc;

    %Get the last column of points
    label = find([calibration_blobs.uc] > image_size_u/2+image_size_u/3);
    % Get the third point - top right
    Pb(1, 3) = calibration_blobs(label(1)).uc;
    Pb(2, 3) = calibration_blobs(label(1)).vc;
    % Get the fourth point - bottom right
    Pb(1, 4) = calibration_blobs(label(3)).uc;
    Pb(2, 4) = calibration_blobs(label(3)).vc;

    % Points of the 4 corner blue markers
    %Q = [20 380; 20 20; 380 380; 380 20;];
    Q = [345 290; 20 290; 345 20; 20 20;];
    H = homography(Pb, Q');

end

function output = find_blobs(worksheet_data, blob_data)

    index = zeros(3,1);

    for i=1:size(blob_data)
        for j=1:size(worksheet_data)
            if blob_data(i,1) == worksheet_data(j,1) && blob_data(i,4) == worksheet_data(j,4) && blob_data(i,5) == worksheet_data(j,5)
                index(i) = j;
                break;
            end
        end
    end
    output = index;
end

function output = classify_blobs(blobs, colour, worksheet)

    output = zeros(length(blobs), 5);
    % Loop through all of the shapes and check the circularity
    % Add results to b_shape
    for i=1:length(blobs)
        if blobs(i).circularity > 0.9 %Check for circle
            output(i,1) = 2;
        elseif blobs(i).circularity > 0.75 %Check for square
            output(i,1) = 0;
        elseif blobs(i).circularity > 0.55 %Check for triangle
            output(i,1) = 1;
        else
        end
    end

    bounding_box_total_area = 0;
    %Loop through and at the Uv coord to the dataoutput
    for i=1:length(blobs)
        output(i,2) = blobs(i).uc;
        output(i,3) = blobs(i).area;
        output(i,5) = colour(i); % 1 if the shape us red
        bounding_box_total_area = bounding_box_total_area + blobs(i).bboxarea;
    end

    %If the worksheet, classify size based on shape
    total_area_circle = 0;
    circle_count = 0;
    total_area_square = 0;
    square_count = 0;
    total_area_triangle = 0;
    triangle_count = 0;

    if worksheet == 1
        for i=1:length(blobs)
            if output(i,1) == 0%If shape is a square
                total_area_square = total_area_square + output(i,3);
                square_count = square_count + 1;
            elseif output(i,1) == 1%If shape is a triangle
                total_area_triangle = total_area_triangle + output(i,3);
                triangle_count = triangle_count + 1;
            elseif output(i,1) == 2%If shape is a circle
                total_area_circle = total_area_circle + output(i,3);
                circle_count = circle_count + 1;
            end
        end
        half_way_circle = total_area_circle/circle_count;
        half_way_square = total_area_square/square_count;
        half_way_triangle = total_area_triangle/triangle_count;
    else
        half_way = bounding_box_total_area / length(blobs);
    end

    %Loop through and assign large all small to each shape    
    if worksheet == 1
        for i=1:length(blobs)
            if output(i,1) == 0%If shape is a square
               if output(i,3) >= half_way_square
                   output(i,4) = 1;
               end
            elseif output(i,1) == 1%If shape is a triangle
               if output(i,3) >= half_way_triangle
                   output(i,4) = 1;
               end
            elseif output(i,1) == 2%If shape is a circle
               if output(i,3) >= half_way_circle
                   output(i,4) = 1;
               end
            end
        end
    else
        for i=1:length(blobs)
            if blobs(i).bboxarea >= half_way
                output(i,4) = 1;
            end
        end

    end

    %If not the worksheet dont sort
    if worksheet == 0
        output = sortrows(output, 2);
    end
end

function print_data(data, homography)
    for i=1:3
        %Shape
        shape = data(i,1);
        if shape == 0
            shape_string = 'Square';
        elseif shape == 1
            shape_string = 'Triangle';
        elseif shape == 2
            shape_string = 'Circle';
        end
        
        %Size
        size = data(i,4);
        if size == 1
            size_string = 'Large';
        elseif size == 0
            size_string = 'Small';
        end
        
        %Color
        color = data(i,5);
        if color == 0 
            color_string = 'Green';
        elseif color == 1
            color_string = 'Red';
        end
        
        %Homography coordinates
        x = homography(1,i);
        y = homography(2,i);
        fprintf('%s\t %s\t %s\t %.0fmm %.0fmm\n', shape_string, size_string, color_string, x, y);
    end
end
%% Defined functions for arm movement
%Pick, Drop
function drop(claw, x, y)
    %Get there
    angle1 = pt2angle(x, y, 70);
    move_to(claw, angle1);
    %Bow
    pause(2);
    angle2 = pt2angle(x, y, 10);
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
    pause(2);
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
    
    joint_angles = [angles(1)-4, angles(2), angles(3), angles(4)+4, claw_position];
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

