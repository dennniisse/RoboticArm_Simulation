classdef GetUR3 < handle
    %when accessing properties in the functions make sure to do
    %"self.<property>"
    properties
        model
        environment = false; %toggle
        stepRads %CalculateAndPlotWorkspace, input value: radians to iterate through
        robotBase = eye(4);
    end
    properties(Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
        baseUR3 = zeros(1,3); %user input, used to update model.base locations
        endEffectorBase = eye(4); %location of endEffectorBase
        gripper;
        % brick variables
        brickLocation = zeros(9,3);
        wallLocation = zeros(9,3);
        bric;
        steps = 50;
        maximumReach;
        volume
        L = SingleInstance.Logger;;
        
    end
    
    methods
        %% (Constructor)
        function self = GetUR3(robotBase, environment)
            diary DenisseFernandez13214489
            self.L = log4matlab('Fernandez13214489.log');
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROBOT START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'); 
            pause(1);
            self.baseUR3 = robotBase(1:3,4)'; %copy to a private variable to prevent user from accidentally changing 
                                              %copy only column 4 and the
                                              %rows 1:3, transpose to list
                                              %numbers horizontally
            self.L.mlog = {self.L.DEBUG,'GetUR3',['The new base for the UR3 is: ',self.L.MatrixToString(self.baseUR3)]};
            % Set workspace based on environment
            self.workspace = [(self.baseUR3(1)-2.3), (self.baseUR3(1)+2) ...
            (self.baseUR3(2)-4), (self.baseUR3(2)+6), ...
            (self.baseUR3(3)-1), (self.baseUR3(3)+1)];  
            disp(['The workspace dimensions are: ', '[',num2str(self.workspace),']']);
            self.L.mlog = {self.L.DEBUG,'GetUR3',['The new workspace dimensions are: ',self.L.MatrixToString(self.workspace)]};
            self.environment = environment;                        
            %%%%%%%%%%%%%%%%%%%%%%%% Plot: Environment -> Robot -> Gripper -> Bricks %%%%%%%%%%%%%%%%%%%%%%%%
            if self.environment == true
            self.PlotEnvironment();
            end
            hold on;
            % Plot Robot
            self.GetUR3Robot();
            % Colour Robot
            self.PlotAndColourRobot();    
            % Initialise animate delay to 0
            self.model.delay = 0;
            % Plot and Colour Gripper.
            self.endEffectorBase = self.model.fkine(self.model.getpos());
            self.L.mlog = {self.L.DEBUG,'GetUR3',['The base of the gripper is: ',self.L.MatrixToString(self.endEffectorBase)]};
            self.gripper = GetGripper(self.endEffectorBase,self.workspace);
            % Plot Brick and obtain current location of the bricks and location of the wall
            self.bric = GetBricks(self.model.base);            
            self.brickLocation = self.bric.GetBrickLocation();     
            self.L.mlog = {self.L.DEBUG,'GetBricks',['The current location of all the bricks on the table is: '...
                                            ,self.L.MatrixToString(self.brickLocation)]};
            self.wallLocation = self.bric.GetWallLocation();
            self.L.mlog = {self.L.DEBUG,'GetBricks',['The expected location of the wall is: ',self.L.MatrixToString(self.wallLocation)]};
            disp('Environment, UR3 and bricks plotted. Press Enter to continue');
            %pause();   

            %%%%%%%%%%%%%%%%%%%%%%%% Begin Mission %%%%%%%%%%%%%%%%%%%%%%%%           
            self.MakeWall();
            self.CalculateAndPlotWorkspace();
            self.GetVolume();
            
            %%%%%%%%%%%%%%%%%%%%%%%% Bonus Demonstration %%%%%%%%%%%%%%%%%%%%%%%%  
            hold off;
            self.gripper.DemonstrateGripper();
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DEMONSTRATION COMPLETED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
            diary off
        end
        
        %% GetUR3Robot 
        function GetUR3Robot(self)
            disp('Creating DH Parameters of UR3');
            pause(0.001);
            name = ['Assessment1_13214489',datestr(now,'yyyymmddTHHMMSSFFF')];
            % dh = [THETA D A ALPHA SIGMA OFFSET]
            L(1) = Link([pi    0      0   pi/2    1   0]);
            L(2) = Link([0    0.1519  0   pi/2    0   0]);
            L(3) = Link([0    0   -0.24365   0    0   0]);
            L(4) = Link([0    0   -0.21325   0    0   0]);
            L(5) = Link([0    0.11235     0   pi/2    0   0]);
            L(6) = Link([0    0.08535     0   -pi/2    0   0]);
            L(7) = Link([0    0.0819      0   0   0   0]);
            
            L(1).qlim = [-0.8 0];
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-360 360]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(7).qlim = [-360 360]*pi/180;
            
            
            self.model = SerialLink(L,'name',name);
            %Rotate the robot and correct orientation                     
            self.model.base = self.model.base*trotx(pi/2) * troty(pi/2);
            %Adjust the base using user input, because UR3 has been
            %rotated, its local axis differs from the global axis
            %To move robot in global axis transl(y,z,x)
            self.model.base = self.model.base * transl([[self.baseUR3(2),self.baseUR3(3),self.baseUR3(1)]]);
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)
            %% Obtain 3D model of UR3
            disp('Plotting and Colouring in UR3');
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
                
            end
            
            %% Plot UR3 as 3D
            % plot3d(UR3,0,'noarrow','workspace',workspace);
            q = zeros(1,7);
            q = [0 -90*pi/180 -45*pi/180 20*pi/180 -90*pi/180 -90*pi/180 0];
            self.model.plot3d(q,'workspace',self.workspace);
            % UR3.teach();
            % Note the function below will make the graphics sharper
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            %% Colour UR3
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name); %findobj: find graphics objects with
                                                    %specific properties
                                                    %'Tag': a property name, therefore
                                                    %it's finding objects whose Tag is
                                                    %UR3
                                                    %h will return the all objects in
                                                    %the hierarchy that have their Tag
                                                    %property set to value 'UR3'
                h = get(handles,'UserData');        %get: returns the value for 'UserData'.
                                                    %h is a structure (see OneNote or
                                                    %print onto cmd)
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ... %%as h is a structure we access h.link and iterate
                        , plyData{linkIndex+1}.vertex.green ...                                         %%through each link and obtain its colour
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            
        end
        %% PlotEnvironment
        function PlotEnvironment(self)
            disp('Plotting Environment');
            offset = -0.67; %adjusting the height of environment so UR3 is placed on top of the table
            [f,v,data] = plyread('Environment2.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Plot the environment, account for any changes in the UR3's base
            trisurf(f,v(:,1)+ self.baseUR3(1)...
                , v(:,2) + self.baseUR3(2)...
                , v(:,3) + offset + self.baseUR3(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');   
            
            hold on;
        end      
        %% Work and Make Wall
        function MakeWall(self) 
            disp('Building Wall');
            gripperOffset = 0.2; % offset the brick location from z-axis to account for gripper 
            % Get UR3 current end effector pose and the location of the first brick 
            self.L.mlog = {self.L.DEBUG,'MakeWall',['The end effector location has a gripper offset of 0.2. Therefore, the real location of the end effector is z - 0.2']};
            for brickIndex = 1:1:9
                %%%%%%%%%%%%%%%%%%%%%%%% Go to Brick %%%%%%%%%%%%%%%%%%%%%%%%
                disp(['Moving to brick number ', num2str(brickIndex)]);                
                currentPos = self.model.getpos(); % [J1 J2 J3 J4 J5 J6 J7]
                x = self.brickLocation(brickIndex,1);
                y = self.brickLocation(brickIndex,2);
                z = self.brickLocation(brickIndex,3) + gripperOffset;
                getBrick = eye(4)*transl([x y z])*troty(pi);
                % Use inverse kinematics to determine the pos required to reach the brick
                newQ = self.model.ikcon(getBrick,currentPos);
                s = lspb(0,1,self.steps); %linear segment with parabolic blend, go from s0 to sf in m steps
                qMatrix = nan(self.steps,7);
                for i = 1:self.steps
                    qMatrix(i,:) = (1-s(i))*currentPos + s(i)*newQ;
                    self.model.animate(qMatrix(i,:));
                    % Gripper move
                    endEffector = self.model.fkine(self.model.getpos());
                    self.gripper.transformGripper(self.steps,endEffector,true);
                    drawnow();
                end 
                self.L.mlog = {self.L.DEBUG,'MakeWall',['The end effector position as it reaches for brick ',num2str(brickIndex),'is: ',self.L.MatrixToString(endEffector)]};
                currentPos = self.model.getpos();
                disp(['- The calculated joint position is: ','[', num2str(newQ), ']']);                
                disp(['  the current and real position is: ','[' ,num2str(currentPos),']']);
                disp(['- The end effector position is: ','[', num2str((endEffector(1:3,4)'-[0,0,gripperOffset])), ']']); % compare end effector postion with the recorded brickLocation
                disp(['  the brick location is: ','[' ,num2str(self.brickLocation(brickIndex,:)),']']);                
                %%%%%%%%%%%%%%%%%%%%%%%% Place Wall %%%%%%%%%%%%%%%%%%%%%%%%
                disp(['Making wall, placing brick number ', num2str(brickIndex), ' down']); 
                self.L.mlog = {self.L.DEBUG,'MakeWall',['The expected position of the brick is ',num2str(brickIndex),'is: ',self.L.MatrixToString(self.wallLocation(brickIndex,:))]};
                x = self.wallLocation(brickIndex,1);
                y = self.wallLocation(brickIndex,2);
                z = self.wallLocation(brickIndex,3)+gripperOffset; %does not require a gripper offset
                placeWall = eye(4)*transl([x y z])*troty(pi)*trotz(pi/2);
                newQ = self.model.ikcon(placeWall,currentPos);
                s = lspb(0,1,self.steps);
                qMatrix = nan(self.steps,7);
                for i = 1:self.steps
                    qMatrix(i,:) = (1-s(i))*currentPos + s(i)*newQ;
                    self.model.animate(qMatrix(i,:));
                    % Gripper move
                    endEffector = self.model.fkine(self.model.getpos());
                    self.gripper.transformGripper(self.steps,endEffector,false);
                    % Brick move
                    self.bric.PlotWall(brickIndex,endEffector);
                    drawnow();
                end
                currentPos = self.model.getpos();
                disp(['- The calculated joint position is: ','[', num2str(newQ), ']']);
                disp(['  the current and real position is: ','[' ,num2str(currentPos),']']);
                disp(['- The expected brick location is: ','[', num2str(self.wallLocation(brickIndex,:)), ']']);
                disp(['  the current brick location is: ','[' , num2str((endEffector(1:3,4)'-[0,0,gripperOffset])),']']);
                self.gripper.OpenGripper();
                self.L.mlog = {self.L.DEBUG,'MakeWall',['The end effector position as it drops brick ',num2str(brickIndex),'is: ',self.L.MatrixToString(endEffector)]};
                self.L.mlog = {self.L.DEBUG,'MakeWall',['The current position of the brick is ',num2str(brickIndex),'is: ',self.L.MatrixToString((endEffector(1:3,4)'-[0,0,gripperOffset]))]};
                
                poseChoice = 0;
                if brickIndex > 4
                    poseChoice = 1;                    
                end
                self.ResetPose(poseChoice);                              
            end
            disp('Wall Complete. Press enter to continue.');
            %pause();
            
        end
        %% Return robot to selected pose
        function ResetPose(self,poseChoice) % 0 for starting position, 1 for new pose       
            intPos = self.model.getpos();
            pose = poseChoice;
            switch(pose)
                case 0
                    finalPos = [-0.8 -90*pi/180 -45*pi/180 20*pi/180 -90*pi/180 -90*pi/180 0];                    
%                     finalPos = deg2rad([0 -0.2 -130 100 -90 -90 0]);
                    % calculations to go from current pose -> final pose
                    s = lspb(0,1,self.steps);
                    qMatrix = nan(self.steps,7);
                    for i = 1:self.steps
                        qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                        self.model.animate(qMatrix(i,:));
                        endEffector = self.model.fkine(self.model.getpos());
                        self.gripper.transformGripper(self.steps,endEffector,false);
                        drawnow();
                    end
                case 1
%                     finalPos = deg2rad([-0.7 0 -60 100 -130 -90 0]);
                    finalPos = [0 90*pi/180 -45*pi/180 20*pi/180 -90*pi/180 -90*pi/180 0];
                    % calculations to go from current pose -> final pose
                    s = lspb(0,1,self.steps);
                    qMatrix = nan(self.steps,7);
                    for i = 1:self.steps
                        qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                        self.model.animate(qMatrix(i,:));
                        endEffector = self.model.fkine(self.model.getpos());
                        self.gripper.transformGripper(self.steps,endEffector,false);
                        drawnow();
                    end
            end
        end
        %% Calculate and Plot UR3 Workspace
        function CalculateAndPlotWorkspace(self)
            hold on;
%             prompt = "Plotting Workspace. Enter the angle in degrees to sample the joint values: ";
            stepRads = 15;
            stepRads = deg2rad(stepRads);
            qlim = self.model.qlim;
            q1steps = abs(qlim(1,1))/((360*2)/rad2deg(stepRads)); %change rad to linear steps
            
            
            % Assign the storage required for the pointCloud as it will more computationally efficienct if we do
            pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic
            
            if(stepRads < 1.75) % if sample is less than 100
                % We iterate through 3 for loops to store position of the end effector, noting q4 = q7 are consistent
                disp('Sampling joints 1 - 3');
                for q1 = qlim(1,1):q1steps:qlim(1,2)
                    for q2 = qlim(2,1):stepRads:qlim(2,2)
                        for q3 = qlim(3,1):stepRads:qlim(3,2)
                            q4=0; q5=deg2rad(-90); q6=deg2rad(-90); q7=0; %this keeps robot straight up, point cloud is to show robots reach
                            q = [q1,q2,q3,q4,q5,q6,q7];                   % can ignore the inside of the plot cloud
                            tr = self.model.fkine(q); %using joint positions, find end effector transform and store the x,y,z location
                            flag = self.model.base();
                            if (tr(3,4) > flag(3,4))% if the z coordinate is greater than the z coordinate of the base, plot. Disregard points below table
                                pointCloud(counter,:) = tr(1:3,4)'; % the apostrophe is to transpose the matrix in the form the point cloud is expecting
                                counter = counter + 1;
                            end
                        end
                    end
                end
            end
            if(stepRads > 1.75) % if sample is greater than 100
                disp('Sampling joints 1 - 6');
                for q1 = qlim(1,1):q1steps:qlim(1,2)
                    for q2 = qlim(2,1):stepRads:qlim(2,2)
                        for q3 = qlim(3,1):stepRads:qlim(3,2)
                            for q4 = qlim(4,1):stepRads:qlim(4,2)
                                for q5 = qlim(5,1):stepRads:qlim(5,2)
                                    for q6 = qlim(6,1):stepRads:qlim(6,2)
                                        q7=0;
                                        q = [q1,q2,q3,q4,q5,q6,q7];
                                        tr = self.model.fkine(q);
                                        flag = self.model.base();
                                        if (tr(3,4) > flag(3,4))% if the z coordinate is greater than the z coordinate of the base, plot. Disregard points below table
                                            pointCloud(counter,:) = tr(1:3,4)'; % the apostrophe is to transpose the matrix in the form the point cloud is expecting
                                            counter = counter + 1;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end 
            display(['After ',num2str(toc),' seconds, completed 100% of poses']);
            % Create a 3D model showing where the end effector can be over all these samples.
            plotCloud = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            disp('Press enter to continue, note this will delete point cloud');
            %pause();
            delete(plotCloud);
        end   
        %% Calculate maximum reach and volume
        function [volume] = GetVolume(self)
            self.volume = ((4/3)*pi*(0.5^3)+(pi*(0.5^2)*0.8))/2 ; % uses volume of a capsule
            volume = self.volume;
            disp('Volume data of UR3 is as follows: ');
            disp(['- The maximum reach of the UR3 without linear rail is ', num2str(0.5), 'm']);
            disp(['- The maximum reach of the UR3 with the linear rail is ', num2str(1.3), 'm'...
                'along the x-axis and ',num2str(0.5), 'm along the y-axis and z-axis']); % 0.5 + 0.8 = 1.3m => UR3maxReach + linearRailLength
            disp(['- The workspace volume of the UR3 is: ',num2str(self.volume),'m^3']);
        end
        
    end
end