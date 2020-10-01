%% Rose Gebhardt - Drone Control Homework 1 - Question 4

clearvars; close all; clc;

%% Part A

% Define battery coordinates in the body frame
p_b = [0.2; 0; 0];

% Define height coordinates in the inertial frame
height = [0; 0; 10];

% Define euler angles
Angles = deg2rad([2;10;20]); % yaw, pitch, roll

% Functions for each rotation
Rv1_v = @(yaw) [cos(yaw), sin(yaw), 0; -sin(yaw), cos(yaw), 0; 0, 0, 1];
Rv2_v1 = @(pitch) [cos(pitch), 0, -sin(pitch); 0, 1, 0; sin(pitch), 0, cos(pitch)];
Rb_v2 = @(roll) [1, 0, 0; 0, cos(roll), sin(roll); 0, -sin(roll), cos(roll)];
Rb_v = @(yaw,pitch,roll) Rb_v2(roll)*Rv2_v1(pitch)*Rv1_v(yaw);

% Battery coordinates in the inertial frame
ans_a = Rb_v(Angles(1),Angles(2),Angles(3))'*p_b + height;

%% Part B

% Define velocity of the body in the body frame
v_body = [15; 1; 0.5];

% Velocity in inertial frame
ans_b = Rb_v(Angles(1),Angles(2),Angles(3))'*v_body;

%% Part C

gamma = Angles(2) - atan2(v_body(3),v_body(1));
ans_c = rad2deg(gamma);

%% Part D

alpha = atan2(v_body(3),v_body(1));
ans_d = rad2deg(alpha);

%% Part E

heading = Angles(1);
beta = asin(v_body(2)/sqrt(v_body(1)^2 + v_body(2)^2 + v_body(3)^2));
ans_e_course = rad2deg(beta);
ans_e_heading = rad2deg(heading);


