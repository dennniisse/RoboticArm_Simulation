classdef GetGripper < handle
    properties
        modelRight;
        modelLeft;
        endEffectorUR3 = eye(4);
    end
    
    properties(Access = private)
        workspace = zeros(1,6);
        name = 'gripper';
        gripperBase = eye(4);
         
        
    end
    
    methods
        %% Constructor: takes input of UR3's EndEffector Location and makes it the end effector's base
        function self = GetGripper(endEffectorUR3,workspace)
            self.workspace = workspace;
            %store EndEffectorUR3's base 
            hold on;
            self.gripperBase = endEffectorUR3; %(1:3,4)'; %stores only the x,y,z values horizontally
            self.PlotBaseAndRightFinger();            
            self.PlotLeftFinger();
            % initialise all animate delays to 0 to speed up animation 
            self.modelLeft.delay = 0;
            self.modelRight.delay = 0;
        end
        
        %% Plot and Colour Base and Right Finger
        function PlotBaseAndRightFinger(self)
            L(1) = Link([0 0 0 0 1 0]);
            L(2) = Link([0   0    0.01   0   0   0]);
            L(1).qlim = [0 0]*pi/180; %make base static
            L(2).qlim = [-5 10]*pi/180; %trial and error to figure out the limit using .teach()
            self.modelRight = SerialLink(L,'name','gripperRight');
            self.modelRight.base =  self.gripperBase * troty(pi/2);%self.modelRight.base * transl([[self.gripperBase(1), self.gripperBase(2), self.gripperBase(3)]]);
            
            %Plot annd Colour Gripper
            for linkIndex = 1:self.modelRight.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['gripper_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.modelRight.faces{linkIndex + 1} = faceData;
                self.modelRight.points{linkIndex + 1} = vertexData;
            end
            q = [0,-5]*pi/180; % gripper open as wide as possible
            self.modelRight.plot3d(q,'workspace',self.workspace);
%             self.modelRight.teach();          
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            for linkIndex = 0:1
                handles = findobj('Tag', self.modelRight.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
%                     disp(ME_1);
                    continue;
                end
            end
        end
        
        %% Plot and Colour Left Finger
        function PlotLeftFinger(self)
            L = Link([0   0    0.01   0   0   0]);
            L.qlim = [-10 5]*pi/180;
            self.modelLeft = SerialLink(L,'name','gripperLeft');
            self.modelLeft.base = self.gripperBase* troty(pi/2);%self.modelLeft.base * transl([[self.gripperBase]]);
            
            % Plot Left Finger 
            [ faceData, vertexData, plyData{2} ] = plyread(['gripper_3.ply'],'tri'); %#ok<AGROW>
            self.modelLeft.faces{2} = faceData;
            self.modelLeft.points{2} = vertexData;
            self.modelLeft.plot3d(5*pi/180,'workspace',self.workspace,'arrow');
            
            % Colour Left Finger   
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end                     
            handles = findobj('Tag', self.modelLeft.name); 
            h = get(handles,'UserData');
            try
                h.link(2).Children.FaceVertexCData = [plyData{2}.vertex.red ...
                    , plyData{2}.vertex.green ...                                  
                    , plyData{2}.vertex.blue]/255;
                h.link(2).Children.FaceColor = 'interp';
            catch ME_1
%                 disp(ME_1);
            end
        end
        
        %% Transform Gripper
        function transformGripper(self,steps,endEffectorUR3,gripperClose) %true close gripper
            %transform Base
            gripperBase = endEffectorUR3;
            if (gripperClose == true)
            q = deg2rad(10/steps); % the distance the gripper moves is already set (15deg). 
                                        % Therefore, using steps the UR3 is moving from, the distance at each step can be computer
            rightQ = self.modelRight.getpos() + q ; %return q = [q1 q2], move towards +ve 5deg to close
            leftQ = self.modelLeft.getpos() - q; %return q = [q1], move towards -ve 10deg to close            
            self.modelRight.base = gripperBase* troty(pi/2)* trotx(pi/2);
            self.modelLeft.base = gripperBase* troty(pi/2)* trotx(pi/2);            
            %Move gripper            
                self.modelRight.animate([0,rightQ(2)]); %move gripper
                self.modelLeft.animate(leftQ); %move gripper
            end
            if (gripperClose == false)
                rightQ = self.modelRight.getpos();
                leftQ = self.modelLeft.getpos();
                self.modelRight.base = gripperBase* troty(pi/2)* trotx(pi/2);
                self.modelLeft.base = gripperBase* troty(pi/2)* trotx(pi/2);
                self.modelRight.animate([0,rightQ(2)]); %move gripper
                self.modelLeft.animate(leftQ); %move gripper                
            end
        end
        
        %% Open Gripper
        function OpenGripper(self)
            rightQ = [0,-5]*pi/180;
            leftQ = 5*pi/180;
            self.modelRight.animate([0,rightQ(2)]); %move gripper
            self.modelLeft.animate(leftQ); %move gripper 
        end 
        
        %% Gripper Demo, demonstration to show fingers close at the same tie
        function DemonstrateGripper(self)
            hold off;
            disp('To begin gripper demonstration, press enter');
            %%%%pause();
            q = zeros(1,1);
            rightQ = zeros(1,2);
            leftQ = zeros(1,1);
            % dh = [THETA D A ALPHA SIGMA OFFSET]
            n= 0.5;
            workspace = [-n n -n n -0.3 n];
            L0 = Link([0 0 0 0 1 0]);
            L1 = Link([0   0    0.01   0   0   0]);
            L0.qlim = [0 0]*pi/180;
            L1.qlim = [-10 5]*pi/180;
            fLeft = SerialLink([L0 L1],'name','fLeft');
            
            name = 'gripper';
            
            for linkIndex = 1:fLeft.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['gripper_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                fLeft.faces{linkIndex + 1} = faceData;
                fLeft.points{linkIndex + 1} = vertexData;
                
            end
            J1 = [0 -10]*pi/180;
            fLeft.plot3d(J1,'workspace',workspace);
            fLeft.teach();
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            for linkIndex = 0:1
                handles = findobj('Tag', fLeft.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    continue;
                end
            end
            hold on;
            L2 = Link([0   0    0.01   0   0   0]);
            L2.qlim = [-5 10]*pi/180;
            fRight = SerialLink(L2,'name','fRight');
            [ faceData, vertexData, plyData{2} ] = plyread(['gripper_3.ply'],'tri'); %#ok<AGROW>
            fRight.faces{2} = faceData;
            fRight.points{2} = vertexData;
            
            
            %% Plot as 3D
            q = 10*pi/180;
            fRight.plot3d(q,'workspace',workspace,'arrow','ortho');
            fRight.teach();
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            %% Colour UR3
            
            handles = findobj('Tag', fRight.name);
            h = get(handles,'UserData');
            try
                h.link(2).Children.FaceVertexCData = [plyData{2}.vertex.red ... %%as h is a structure we access h.link and iterate
                    , plyData{2}.vertex.green ...                                         %%through each link and obtain its colour
                    , plyData{2}.vertex.blue]/255;
                h.link(2).Children.FaceColor = 'interp';
            catch ME_1
                disp(ME_1);
            end
            steps = 20;
            q = 20/steps; 
            for i = 1:q:20
                rightQ = fLeft.getpos()+(q*pi/180); %return q = [q1 q2]
                leftQ = fRight.getpos()-(q*pi/180); %return q = [q1]
                %transform Base
                gripperBase = eye(4);
                fLeft.animate([0,rightQ(2)]); % move towards +ve 5deg to close
                % move towards -ve 10deg to close
                fRight.animate([leftQ]);
                drawnow();
                
            end
            
        end
    end
end
