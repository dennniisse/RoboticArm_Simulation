classdef GetBricks < handle
    properties 
        model
        baseUR3 = eye(4);
    end
    
    properties (Access = private)
        brick1Base = zeros(1,3); % x y z of brick1
        brickVertexCount;
        brickLocation = zeros(9,3); % stores value of all bricks in a matrix 9 x 3
        wallLocation = zeros(9,3) % stores values of wall
        brick_h = [patch patch patch patch patch patch patch patch patch]; % brick handle
        v;
    end 
    
    methods 
        %% Constructor 
        function self = GetBricks(baseUR3)               
            % Obtain location of UR3 base [x y z], transpose from column2row. This will be brick 1's base. 
            self.brick1Base = baseUR3(1:3,4)';
            self.PlotBricks();     
%             self.SetBrick9Location();  
            self.UpdateWallLocation(); % initialise WallLocation
        end
        
        function PlotBricks(self)
            % Obtain ply data
            [f,self.v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');            
            % scale vertex colour
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Get brick vertices count, will be used  to transform brick location
            self.brickVertexCount = size(self.v,1); %Obtain row size only 
            % Create x,y,z offset for bricks, as trisurf will spawn bricks at 0 0 0
            xOffset = 0.20; % change for each brick, - 0.15 increment
            yOffset = 0.35; % keep this consistent for all bricks 
            zOffset = 0; % keep this consistent for all bricks
            % Update brick 1's location with the offsets
            self.brickLocation(1,:) = self.brick1Base + [xOffset yOffset zOffset];
            xOffset = - 0.15; % reset and increment xOffset
            % Store location of bricks 2 - 9, reliant on brick1's location
            for brickCount = 2:1:9
                self.brickLocation(brickCount,:) = [(self.brickLocation(1,1)+xOffset) self.brickLocation(1,2) self.brickLocation(1,3)];    
                xOffset = xOffset - 0.15; %increment xOffset
            end
            % Plot bricks using trisurf             
            for brickIndex = 1:1:9
            self.brick_h(brickIndex) = trisurf(f,self.v(:,1)+self.brickLocation(brickIndex,1)...
                , self.v(:,2)+self.brickLocation(brickIndex,2)...
                , self.v(:,3)+self.brickLocation(brickIndex,3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            end
            
             self.brickLocation(8,:) = [(self.brickLocation(1,1)+0.01) self.brickLocation(1,2) self.brickLocation(1,3)];
             self.brickLocation(9,:) = [(self.brickLocation(1,1)+0.01) self.brickLocation(1,2) self.brickLocation(1,3)];
             
            
        end
        
        function [getBrickLocations] = GetBrickLocation(self) % return brickLocation
            getBrickLocations = self.brickLocation;
        end
        
        function SetBrick9Location(self)
            prompt = "Would you like to change location of a brick? Enter [y/n]: ";
            ans = input(prompt,"s");
            if ans == 'y'
                prompt = "Enter an x,y coordinate to change the location of one brick. Enter x: ";
                newLocationX = input(prompt);
                prompt = "Enter y: ";
                newLocationY = input(prompt);
                self.brickLocation(9,1) = newLocationX;
                self.brickLocation(9,2) = newLocationY;
                newLocation = [eye(4) * transl([self.brickLocation(9,1),self.brickLocation(9,2),self.brickLocation(9,3)])*  [self.v,ones(self.brickVertexCount,1)]']';
                self.brick_h(9).Vertices = newLocation(:,1:3);
                disp(['The new location of brick 9 is ','[',num2str(self.brickLocation(9,1)),' ',num2str(self.brickLocation(9,2)),' ',num2str(self.brickLocation(9,3)),']']);
            elseif ans == 'n'
                disp('Brick remains unchanged');
            end
        end
        
        % Wall location is reliant on UR3's base 
        function UpdateWallLocation(self)
            xOffset = 0.133; % Offset from base of UR3
            yOffset = -0.35; % Offset from base of UR3
            zOffset = 0;
            % Store brick location from brick 2 onwards
            self.wallLocation(1,:) = self.brick1Base(1,:) + [xOffset yOffset zOffset];
            xOffset = -0.133; % change x axis for width of brick
            for brickCount = 2:1:3 % y and z constant, x changes
                self.wallLocation(brickCount,:) =  [(self.wallLocation(1,1)+xOffset) self.wallLocation(1,2) self.wallLocation(1,3)];
                xOffset = xOffset - 0.133; %increment xOffset
            end
            % Change brick4-6 location to place bricks on 2nd row, y stays constant x returns to 0.067
            zOffset = 0.034; % Increase by height of brick
            self.wallLocation(4,:) =  [(self.wallLocation(1,1)), (self.wallLocation(1,2)), (self.wallLocation(1,3)+zOffset)];
            xOffset = -0.133; % change x-axis to width increment
            for brickCount = 5:1:6
                self.wallLocation(brickCount,:) =  [(self.wallLocation(4,1)+xOffset) self.wallLocation(4,2) self.wallLocation(4,3)];
                xOffset = xOffset - 0.133; %increment xOffset
            end
            % Change brick7-9 location to place bricks on 3d row
            zOffset = 2*0.034; % Increase by height of brick
            self.wallLocation(7,:) =  [(self.wallLocation(1,1)) self.wallLocation(1,2) self.wallLocation(1,3)+zOffset]
            xOffset = -0.133; % change x-axis to width increment
            for brickCount = 8:1:9
                self.wallLocation(brickCount,:) =  [(self.wallLocation(7,1)+xOffset) self.wallLocation(7,2) self.wallLocation(7,3)];
                xOffset = xOffset - 0.133; %increment xOffset
            end
%             % Testing purposes, comment out
%             [f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');            
%             vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%             for brickIndex = 1:1:9
%                 brick_h(brickIndex) = trisurf(f,v(:,1)+ self.wallLocation(brickIndex,1)...
%                     , v(:,2)+ self.wallLocation(brickIndex,2)...
%                     , v(:,3)+ self.wallLocation(brickIndex,3)...
%                     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%             end           
        end
        
        function [wallLocation] = GetWallLocation(self)
            wallLocation = self.wallLocation;
        end
        
        function PlotWall(self,brickIndex,endEffectorBase)
            gripperOffset = 0.15;
            newLocation = [endEffectorBase * transl([0,0,gripperOffset])* [self.v,ones(self.brickVertexCount,1)]']';
            self.brick_h(brickIndex).Vertices = newLocation(:,1:3);
        end        
      
        
         
        
        
    end
    
end 