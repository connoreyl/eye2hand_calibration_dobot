classdef dobotMagician < handle
    properties
        %> Robot model
        model;
        
        %> Workspace
        workspace = [-2 2 -2 2 -2 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for DoBot Magician robot simulation
function self = dobotMagician(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace

        
% robot = 
self.GetDoBotRobot();
% robot = 
q = zeros(1,4);
plot(self.model,q);
end

%% GetDoBotRobot
% Given a name (optional), create and return a UR3 robot model
function GetDoBotRobot(self)
    %     if nargin < 1
    % Create a unique name (ms timestamp after 1ms pause)
    pause(0.001);
    name = ['DoBot_',datestr(now,'yyyymmddTHHMMSSFFF')];
    %     end
    
    L1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
    L2 = Link('d',0,'a',0.16828,'alpha',0,'offset',0,'qlim',[deg2rad(0),deg2rad(85)]);
    L3 = Link('d',0,'a',0.190,'alpha',0,'offset',0,'qlim',[deg2rad(-10),deg2rad(95)]);
    L4 = Link('d',0,'a',0.03343,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
    self.model = SerialLink([L1 L2 L3 L4], 'name', 'dobotMagician');
end
    end
end