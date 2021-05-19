% Visual Servoing Code
% Code controls the Dobot Magician using an Intel RealSense D435i Camera
% using AR tags

%% Startup Process

% Reset Matlab
    clear;
    clc;
    close;
% Run the Intel RealSense Camera
    % roslaunch realsense2_camera rs_camera.launch
% Run the ar_tag_toolbox with the parameters supplied in the launch file
    % roslaunch ar_tag_toolbox ar_track_usb_cam.launch
% Run the AR_Convert toolbox for simple MATLAB Integration for AR Tags
    % rosrun ar_convert main
% Run rviz
    % rosrun rviz rviz
% Intialise ROS on MATLAB (rosshutdown for terminating)
    rosshutdown;
    rosinit;
% Run the Dobot MAgician Driver with the robot plugged in
    % sudo chmod a+rw /dev/ttyUSB0
    % roslaunch dobot_magician_driver dobot_magician.launch
    dobot = DobotMagician();
    
%% Home the Dobot

% Set an End-Effector Postion and Rotation
end_effector_position = [0.1,0,0];
end_effector_rotation = [0,0,0];

% Publish the Pose to the Dobot
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
% Wait for dobot to home
pause(3);
    
%% Camera to Dobot Base Transform

% Set up ROS Subscriber for AR Tags and save the array to variable
ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
tag_msg = receive(ARTagSub);

% Get information of the first tag
ar_tag = tag_msg.Poses(1);

% Place AR Tag 1 in front of the robot base
% Offset of the tag to the Dobot base (m)
ar_base_offset = [-0.092, 0.0, 0.13];

% Hover height of the end-effector
ee_hover_height = 0.02;

% Translation component of the AR Tag
t_cam_base = [ar_tag.Position.X,ar_tag.Position.Y,ar_tag.Position.Z]';
% Rotation component of AR Tag
r_cam_base = quat2rotm([ar_tag.Orientation.W,ar_tag.Orientation.X,ar_tag.Orientation.Y,ar_tag.Orientation.Z]);

% Convert R and t into a transform
tr_cam_base = rt2tr(r_cam_base, t_cam_base);
% Add Dobot Base offset
tr_cam_base = tr_cam_base*transl(ar_base_offset(1), ar_base_offset(2), ar_base_offset(3) - ee_hover_height)


%% Dobot Base to End-Effector Transform

% Get the Current Joint State
joints = dobot.GetCurrentJointState';

% Arm lengths (m) with respect to the origin of the robot
arm_lengths = [0.13, 0.135, 0.147];

% Tool tip offsets from control point
arm_offsets = [0.06, 0, -0.06];
    
% Calculate the translation from the robot base to the end effector
x = cos(joints(1)) * (arm_lengths(3) * cos(joints(3)) + arm_lengths(2) * sin(joints(2)) + arm_offsets(1));
y = sin(joints(1)) * (arm_lengths(3) * cos(joints(3)) + arm_lengths(2) * sin(joints(2)) + arm_offsets(2));
z = arm_lengths(1) - arm_lengths(3) * sin(joints(3)) + arm_lengths(2) * cos(joints(2)) + arm_offsets(3);

% Rotation around the Z axis is caused by the angle of q1 (Z,X,Y)
r_base_ee = eul2rotm([joints(1),0,0]);

% The translation is combined from the x, y and z calculated above
t_base_ee = [x, y, z]'

% Determine the transform
tr_base_ee = rt2tr(r_base_ee, t_base_ee);

% Determine the pose of end effector with respect to the camera for debug
debug_cam_ee = tr_cam_base*tr_base_ee;

%% Following AR Tag

% Recieve Tag
tag_msg_2 = receive(ARTagSub);

% Get information of the second tag
ar_tag_2 = tag_msg_2.Poses(2);

% Translation component of the AR Tag
t_cam_tag = [ar_tag_2.Position.X,ar_tag_2.Position.Y,ar_tag_2.Position.Z]';
% Rotation component of AR Tag
r_cam_tag = quat2rotm([ar_tag_2.Orientation.W,ar_tag_2.Orientation.X,ar_tag_2.Orientation.Y,ar_tag_2.Orientation.Z]);

% Convert R and t into a transform
tr_cam_tag = rt2tr(r_cam_tag, t_cam_tag);

%% Determine Base to Tag

% Multiply transforms to get tag in robot base frame
tr_base_tag = inv(tr_cam_base)*tr_cam_tag


% Determine translation for the required end effector pose
t_base_tag = (transl(tr_base_tag))'

% Determine the angle of Joint 1 using atan2
theta = atan2(t_base_tag(2), t_base_tag(1))

% Account for tool offset
arm_offset_function = [arm_offsets(1) * cos(theta) - arm_offsets(2) * sin(theta),...
    arm_offsets(1) * sin(theta) - arm_offsets(2) * cos(theta), arm_offsets(3)]

% Control point in base frame of end effector tip
t_control = t_base_tag - arm_offset_function

%% Move the Dobot to a Position for Control

% Set an End-Effector Postion for the AR Tag
end_effector_position = t_control %t_base_tag
end_effector_rotation = [0,0,0];

% Publish the Pose
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%return;

%% Follow Tag

while true
    % Recieve Tag
    tag_msg_2 = receive(ARTagSub);

    % Get information of the second tag
    try
        ar_tag_2 = tag_msg_2.Poses(2);
    catch
        disp('Only 1 AR Tag Detected!!');
        pause(0.5);
        continue
    end

    % Translation component of the AR Tag
    t_cam_tag = [ar_tag_2.Position.X,ar_tag_2.Position.Y,ar_tag_2.Position.Z]';
    % Rotation component of AR Tag
    r_cam_tag = quat2rotm([ar_tag_2.Orientation.W,ar_tag_2.Orientation.X,ar_tag_2.Orientation.Y,ar_tag_2.Orientation.Z]);

    % Convert R and t into a transform
    tr_cam_tag = rt2tr(r_cam_tag, t_cam_tag);

    % Determine End Effector to Tag
    tr_base_tag = inv(tr_cam_base)*tr_cam_tag

    t_base_tag = transl(tr_base_tag)'
    
    % Determine the angle of Joint 1 using atan2
    theta = atan2(t_base_tag(2), t_base_tag(1))

    % Account for tool offset
    arm_offset_function = [arm_offsets(1) * cos(theta) - arm_offsets(2) * sin(theta),...
        arm_offsets(1) * sin(theta) - arm_offsets(2) * cos(theta), arm_offsets(3)]

    % Control point in base frame of end effector tip
    t_control = t_base_tag - arm_offset_function

    % Move the Dobot to a Position for Control
    % Set an End-Effector Postion
    end_effector_position = t_control;
    end_effector_rotation = [0,0,0];

    % Publish the Pose
    dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
    pause(0.8);
end

%% Debug with Tr Plot

% axis = [-0.5 1 -0.7 0.7 -0.7 0.7];
% 
% trplot(eye(4),'length',0.2, 'axis', axis);
% hold on;
% trplot(tr_cam_base,'color',[1 0 0],'length',0.2);
% hold on;
% trplot(tr_cam_tag,'color',[1 0 1],'length',0.2);
% hold on;
% trplot(debug_cam_ee,'color',[0 1 0],'length',0.2);
