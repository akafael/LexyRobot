%% Class to represent control an two link robot
classdef lexyRobot
    % LEXY_ROBOT functions for robot
    
    properties
        name = 'Lexy Robot'     % Robot Name
        robotModel;             % Model Representation
        robotArduino;           % Harware Connection
        mArduino;               % Arduino Library
        connectionPort;         % Connection Port
        this                    % Reference for the object

        %% DH Parameters (inch, radians)
        % dhLink1 = struct('a',4.534 , 'd',1.86 , 'alpha',0);
        % dhLink2 = struct('a',4.225 , 'd',0.71 , 'alpha',0);
        
        %% DH Parameters (cm , radians)
        dhLink1 = struct('a',11.51 , 'd',4.72 , 'alpha',0);
        dhLink2 = struct('a',10.75 , 'd',1.80 , 'alpha',0);
        
        %% DH Parameters (mm , radians)
        % dhLink1 = struct('a',115.1 , 'd',47.2 , 'alpha',0);
        % dhLink2 = struct('a',107.5 , 'd',18.0 , 'alpha',0);

        %% Robot State Variables
        robotPos = [pi/2 pi/2];         % Current joint angles
        penState = 1;                   % Current Pen State
        isConnected2Arduino = false;    % Status Connection
    end

    properties(Constant=true)
        %% Calibration Constants
        LINK1_PI = pi/0.9;
        LINK2_PI = pi/0.9;

        MAX_SPEED_SERVO = (pi/3)/0.19;  % rad/s
        DT = 0.007;
        
        ANGLE_PEN_UP = 0.4;
        ANGLE_PEN_DOWN = 0.9;
        PEN_UP = 1;
        PEN_DOWN = 0;

        %% Debug Constants
        
    end
    
    methods
    	function obj = lexyRobot()
            obj.robotModel = obj.createModel();
            obj.penState = obj.PEN_UP;
            % obj.robotArduino = obj.connectArduino('/dev/ttyACM0');
            % obj.robotModel.plot(obj.robotPos);
            % obj.moveFast(obj.robotPos);
    	end

    	function printName(obj)
    		disp(obj.name)
    	end

        function moveSync(obj,q1)
            %% Move Simulation and Robot synchronized

            % Check if the angle is allowed for the robot
            if((sum(q1<0) + sum(q1>pi) ) >0)
                msgID = 'ROBOT:AngleOutsideLimit';
                msg = '\tValue outside the joint limit. The angle must be between 0 and pi';
                angleException = MException(msgID,msg);
                throw(angleException);
            end

            % Generate Trajetory:
            q0 = obj.robotModel.getpos();
            N = round(max(abs(q1-q0))/(obj.MAX_SPEED_SERVO*obj.DT));
            qs = jtraj(q0,q1,N);

            for i = 2:N
                q = qs(i,1:2);
                dt = max(abs(qs(i,1:2)-qs(i-1,1:2)))/obj.MAX_SPEED_SERVO;

                % Plot
                obj.robotModel.plot(q);
                
                % Move servos
                if(obj.isConnected2Arduino)
                    try
                        obj.robotArduino.servolink1.writePosition(...
                            (pi - q(1))/obj.LINK1_PI);
                        obj.robotArduino.servolink2.writePosition(...
                            (pi - q(2))/obj.LINK2_PI);

                        q0(1) =  obj.robotArduino.servolink1.readPosition();
                        q0(2) =  obj.robotArduino.servolink2.readPosition();

                        obj.robotPos = q;
                    catch ME
                        obj.arduinoDisp('Unable to move Servos');
                        rethrow(ME);
                    end
                end

                pause(dt);
            end
        end

        function moveFast(obj,q)
            %% Plot final position and then move robot

            % Check if the angle is allowed for the robot
            if((sum(q<0) + sum(q>pi) ) >0)
                msgID = 'ROBOT:AngleOutsideLimit';
                msg = '\tValue outside the joint limit. The angle must be between 0 and pi';
                angleException = MException(msgID,msg);
                throw(angleException);
            end

            % Generate Trajetory:
            q0 = obj.robotModel.getpos();
            qs = jtraj(q0,q,3);

            % Plot
            obj.robotModel.plot(qs,'fps',60);
            
            % Move Servos
            if(obj.isConnected2Arduino)
                try
                    obj.robotArduino.servolink1.writePosition(...
                        (pi - q(1))/obj.LINK1_PI);
                    obj.robotArduino.servolink2.writePosition(...
                        (pi - q(2))/obj.LINK2_PI);

                    q0(1) =  obj.robotArduino.servolink1.readPosition();
                    q0(2) =  obj.robotArduino.servolink2.readPosition();

                    obj.robotPos = q;
                catch ME
                    obj.arduinoDisp('Unable to move Servos');
                    rethrow(ME);
                end
            end
        end
        
        function penState = penUpDown(obj, state)
            if(obj.isConnected2Arduino)
                if state==obj.PEN_UP
                    penState = obj.PEN_UP;
                    obj.robotArduino.servoActuator.writePosition(obj.ANGLE_PEN_UP);
                else
                    penState = obj.PEN_DOWN;
                    obj.robotArduino.servoActuator.writePosition(obj.ANGLE_PEN_DOWN);
                end
            end
        end

        %% Arduino Functions
        function robotArduino = connectArduino(obj,port)
            %% Create the connection with the robot

            % Connect With Arduino
            obj.arduinoDisp(['Connecting with port ',port, '...']);

            try
                delete(instrfind({'Port'},{port}));
                obj.mArduino = arduino(port, 'Uno', 'Libraries', {'Servo'});
                obj.connectionPort = port;

                % Attach Servos
                obj.robotArduino.servolink2 = servo(obj.mArduino,9);
                obj.robotArduino.servolink1 = servo(obj.mArduino,10);
                obj.robotArduino.servoActuator = servo(obj.mArduino,11);

                % Move to initial state
                q = obj.robotPos;
                obj.robotArduino.servolink1.writePosition(...
                        (pi - q(1))/obj.LINK1_PI);
                    obj.robotArduino.servolink2.writePosition(...
                        (pi - q(2))/obj.LINK2_PI);
                obj.penUpDown(obj.PEN_UP);
                
                % Show message
                obj.arduinoDisp([obj.name,' Connected! =D']);
                
                % Return reference for servo motors
                robotArduino = obj.robotArduino;
            catch ME
                obj.arduinoDisp('Connection Error!');
                robotArduino = [];

                if(obj.isConnected2Arduino)
                    rethrow(ME);
                end
            end
        end

        function disconnectArduino(obj)
            %% Disconnect the robot
            delete(instrfind({'Port'},{obj.connectionPort}));
            obj.arduinoDisp([obj.name,' Disconnected! =O']);
        end

        %% Robotic Toolkit Functions
        function model= createModel(obj)
            %% Model Representation using Robotic Toolkit

            obj.robotToolkitDisp('Creating model for Control and Simmulation')

            % Joints Description
            LINK(1) = Revolute('d', obj.dhLink1.d, 'a', obj.dhLink1.a,...
                'alpha', obj.dhLink1.alpha, 'm', 0, 'offset',-pi/2, ...
                'qlim', [0 pi]); % kinematic: joint variable limits

            LINK(2) = Revolute('d', obj.dhLink2.d, 'a', obj.dhLink2.a,...
                'alpha', obj.dhLink2.alpha, 'm', 0, 'offset',0, ...
                'qlim', [0 pi]); % kinematic: joint variable limits

            % Create the Robot Model
            obj.robotModel = SerialLink(LINK, 'name', obj.name);
            obj.robotToolkitDisp(['Model for ',obj.robotModel.name,' created ']);

            model = obj.robotModel;
        end
        
        function drawLetter(obj,char,scale)
            % Get the current position
            q = obj.robotModel.getpos();
            T = obj.robotModel.fkine(q);
            posX = T(1,4);
            posY = T(2,4);
            
            if char == ' '
                dx = 0.5
                q = obj.ikine(posX+dx,posY);
                obj.penUpDown(obj.PEN_UP);
                obj.moveFast(q);
            else
                obj.robotToolkitDisp(['Writing letter ',char]);
                nameFile = ['Letters/nletter',char,'.mat'];
                load(nameFile,'letter');

                letter(:,1) = scale*letter(:,1) + posX;
                letter(:,2) = scale*letter(:,2) + posY;
                letter(:,3) = obj.PEN_DOWN;
                
                % Plot Shape
                for i = 1:size(letter,1)
                    q = obj.ikine(letter(i,1),letter(i,2));
                    obj.moveFast(q);
                    obj.penUpDown(letter(i,3));
                end
                
                % Prepare for the next letter
                posX = max(letter(:,1));
                posY = min(letter(:,2));
                q = obj.ikine(posX,posY);
                % obj.penUpDown(obj.PEN_UP);
                obj.moveFast(q);
            end
        end
        
        function plotLetter(obj,char,handles)
             if char == ' '
                
             else
                fig = gca;
                axes(handles);
                nameFile = ['Letters/nletter',char,'.mat'];
                load(nameFile,'letter');
                
                plot(letter(:,1)',letter(:,2)','b');
                axis off;
                axes(fig);
             end
        end
        
        function drawWord(obj,msg,scale)
            % msg String to Draw
            % scale letter height
            for i = 1:size(msg,2);
                obj.drawLetter(msg(i),scale);
                obj.penUpDown(obj.PEN_UP);
                q = obj.robotModel.getpos();
                obj.moveFast(q);
            end
        end
        
        function drawCurve(obj,curve)
            obj.robotToolkitDisp(['Preparing...',char]);
            obj.penUpDown(obj.PEN_UP);
            
            obj.robotToolkitDisp(['Drawing curve',char]);
            % Plot Curve
            for i = 1:size(curve,1)
                q = obj.ikine(curve(i,1),curve(i,2));
                obj.moveFast(q);
                obj.penUpDown(obj.PEN_DOWN);
            end
            
            obj.penUpDown(obj.PEN_UP);
        end
        
        function q1 = ikine(obj,X,Y)
            %% Inverse Kinematic adapted for this robot
            
            % Get Current postion as initial condition
            qz = obj.robotPos;
            Tz = obj.robotModel.fkine(qz);

            % Generate New Position
            T1 = rt2tr(t2r(Tz),[X Y Tz(3,4)]');

            % Create New Position Using Ikine
            M = [1 1 0 0 0 0];
            q1 = obj.robotModel.ikine(T1,qz,M);
        end

        %% Custom Messages
        function arduinoDisp(obj,text)
            %% Message Output for Arduino Events
            disp(['[Arduino]: ',text]);
        end

        function robotToolkitDisp(obj,text)
            %% Message Output for Robot Toolkit Events
            disp(['[Robotic Toolkit]: ',text]);
        end
    end
end