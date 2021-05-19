classdef ARControl < handle

    
    properties
        ARTagSub;
        tag_msg;
        
        ar_base_offset = [-0.092, 0.0, 0.13];
        ee_hover_height = 0.05;
        
        arm_lengths = [0.13, 0.135, 0.147];
        arm_offsets = [0.06, 0, -0.06];
        
        ar_tag_positions;
        tr_base_ee;
        
        tr_cam_base; % for plotting tr
        tr_cam_tag;  % for plotting tr
        tr_cam_ee;   % for plotting tr
        
        robot1;
        
        followMode = 0;
        
    end
    
    methods
        %%
        function self = ARControl()
            self.ARTagSub = rossubscriber('/tags','geometry_msgs/PoseArray');
            self.tag_msg = receive(self.ARTagSub);
            
            self.robot1 = DobotControl();

        end
        
        %%
        function GetARTagPositions(self)

            ar_tags = receive(self.ARTagSub);
                        
            robot_tag = ar_tags.Poses(1);
            

            t_cam_base = [robot_tag.Position.X, robot_tag.Position.Y, robot_tag.Position.Z]';
            r_cam_base = quat2rotm([robot_tag.Orientation.W, robot_tag.Orientation.X, robot_tag.Orientation.Y, robot_tag.Orientation.Z]);
            
            self.tr_cam_base = rt2tr(r_cam_base, t_cam_base);
            self.tr_cam_base = self.tr_cam_base*transl(self.ar_base_offset(1), self.ar_base_offset(2), self.ar_base_offset(3) - self.ee_hover_height)
            
            self.ar_tag_positions{1} = self.tr_cam_base;
            
            for i = 2 : length(ar_tags.Poses)
                
                ar_tag = ar_tags.Poses(i);
                
                t_cam_tag = [ar_tag.Position.X, ar_tag.Position.Y, ar_tag.Position.Z]';
                r_cam_tag = quat2rotm([ar_tag.Orientation.W, ar_tag.Orientation.X, ar_tag.Orientation.Y, ar_tag.Orientation.Z]);
                
                self.tr_cam_tag = rt2tr(r_cam_tag, t_cam_tag);
                
                self.ar_tag_positions{i} = self.tr_cam_tag;
                
            end
            
            
            disp(self.ar_tag_positions);
        end
        
        %%
        function GetEndEffectorPosition(self)
            
            joints = self.robot1.GetJointStates();
            
            % Calculate the translation from the robot base to the end effector
            x = cos(joints(1)) * (self.arm_lengths(3) * cos(joints(3)) + self.arm_lengths(2) * sin(joints(2)) + self.arm_offsets(1));
            y = sin(joints(1)) * (self.arm_lengths(3) * cos(joints(3)) + self.arm_lengths(2) * sin(joints(2)) + self.arm_offsets(2));
            z = self.arm_lengths(1) - self.arm_lengths(3) * sin(joints(3)) + self.arm_lengths(2) * cos(joints(2)) + self.arm_offsets(3);
            
            % Rotation around the Z axis is caused by the angle of q1 (Z,X,Y)
            r_base_ee = eul2rotm([joints(1),0,0]);
            
            % The translation is combined from the x, y and z calculated above
            t_base_ee = [x, y, z]'
            
            % Determine the transform
            self.tr_base_ee = rt2tr(r_base_ee, t_base_ee);
            
            self.tr_cam_ee = self.tr_cam_base*self.tr_base_ee;
            
        end
    
        %%
        function MoveToTag(self, tag)
            
            % Calculate Transform for Tag to Robot Base
            % Multiply transforms to get tag in robot base frame
            
            tag
            
            tr_base_tag = inv(cell2mat(self.ar_tag_positions(1))) * cell2mat(self.ar_tag_positions(tag+1));
            
            
            % Determine translation fr the required end effector pose
            t_base_tag = (transl(tr_base_tag))';
            
            % Determine the angle of Joint 1 using atan2
            theta = atan2(t_base_tag(2), t_base_tag(1));
            
            % Account for tool offset
            arm_offset_function = [self.arm_offsets(1) * cos(theta) - self.arm_offsets(2) * sin(theta),...
                self.arm_offsets(1) * sin(theta) - self.arm_offsets(2) * cos(theta), self.arm_offsets(3)];
            
            % Control point in base frame of end effector tip
            t_control = t_base_tag - arm_offset_function;
            
            self.robot1.MoveToCartesianPoint(t_control);
        end
        
        %%
        function FollowARTag(self)
            
            if self.followMode == 0
                self.followMode = 1;
            elseif self.followMode == 1
                self.followMode = 0;
            end

            while self.followMode == 1
                
                tag_msg_2 = receive(self.ARTagSub);
                
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
                self.tr_cam_tag = rt2tr(r_cam_tag, t_cam_tag);
                
                % Determine End Effector to Tag
                tr_base_tag = inv(self.tr_cam_base)*self.tr_cam_tag
                
                t_base_tag = transl(tr_base_tag)'
                
                % Determine the angle of Joint 1 using atan2
                theta = atan2(t_base_tag(2), t_base_tag(1))
                
                % Account for tool offset
                arm_offset_function = [self.arm_offsets(1) * cos(theta) - self.arm_offsets(2) * sin(theta),...
                    self.arm_offsets(1) * sin(theta) - self.arm_offsets(2) * cos(theta), self.arm_offsets(3)]
                
                % Control point in base frame of end effector tip
                t_control = t_base_tag - arm_offset_function
                
                % Move the Dobot to a Position for Control
                % Set an End-Effector Postion
                end_effector_position = t_control;
                end_effector_rotation = [0,0,0];
                
                % Publish the Pose
                self.robot1.MoveToCartesianPoint(t_control)
                pause(0.1);
                
            end
            
        end
        
        %%
        function TRPlot(self)

            axis = [-0.5 1 -0.7 0.7 -0.7 0.7];
            
            trplot(eye(4),'length',0.2, 'axis', axis);
            hold on;
            trplot(self.tr_cam_base,'color',[1 0 0],'length',0.2);
            hold on;
            trplot(self.tr_cam_tag,'color',[1 0 1],'length',0.2);
            hold on;
            trplot(self.tr_cam_ee,'color',[0 1 0],'length',0.2);
            
        end
        
        %%
        function numberOfTags = GetNumberOfTags(self)
            numberOfTags = length(self.ar_tag_positions) - 1;
        end
        
        %%
        function tagPositions = GetTagPositions(self)

            for i = 2 : length(self.ar_tag_positions)

                i
                
                position = self.ar_tag_positions{i};
                
                x = num2str(position(1, 4));
                y = num2str(position(2, 4));
                z = num2str(position(3, 4));
                
                str = strcat('X: ', x, ' Y: ', y, ' Z: ', z) 
                
                strArray{i-1} = str;
                
            end

            tagPositions = strArray;
            
        end
        
        
        %%
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

