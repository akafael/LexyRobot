%% Test Function
function mRobot = testLexyRobot()
	% Start Robotic Toolkit
	startup_rvc();

	% Create Robot Object
	mRobot = lexyRobot();
	mRobot.printName();
    
    % Connect Arduino
    mRobot.robotArduino = mRobot.connectArduino('/dev/ttyACM0');
    mRobot.isConnected2Arduino = true;
    
    % Start Simulation
    mRobot.robotModel.plot([pi/2 pi/2]);
    
    %testMove(mRobot);
    %testCalibrationMove(mRobot);
    %testInverseKinematics(mRobot,3,4);
    %testDrawSquare(mRobot);
    %testDrawWord(mRobot,'RAFAEL',3)
    %testSizeLetter(mRobot);
    testPlottingMap();
    %testDrawingPath(mRobot);
    
    % Disconnect Arduino
    mRobot.disconnectArduino();
end

function testMove(mRobot)
    % Test Rotine for the function move
   	% Using Robotic Toolkit (Direct Use)
	% use mRobot.robotModel for the SerialLink object
	q= [0 0]
	mRobot.robotModel.plot(q)


	% Using Arduino (Direct Use)
	% use mRobot.robotArduino to access the Arduino functions
    try
        mRobot.robotArduino.servolink1.writePosition(q(1)/pi);
        mRobot.robotArduino.servolink2.writePosition(q(2)/pi);
    catch
        disp('Unable to move Arm!')
    end
    
    % Using Both combined
    q1 = [pi/2 pi/2];
    mRobot.moveFast(q1);
end

function testCalibrationMove(mRobot)
    % Test Rotine to calibrate the Servo Motors
     q0 = [0 0];
     q1 = [pi pi];
     
     mRobot.move(q0);
     mRobot.move(q1)
end

function testInverseKinematics(mRobot,X,Y)
    q = mRobot.ikine(X,Y)
    mRobot.robotModel.plot(q);
end

function testDrawSquare(mRobot)
    shape = [1 1;1 2;2 2;2 1]

    % Plot Shape
    for i = 1:size(shape,1)
        q = mRobot.ikine(shape(i,1),shape(i,2))
        mRobot.robotModel.plot(q);
        pause(0.5);
    end
    
    % Go Back to the Begining of shape
    q = mRobot.ikine(shape(1,1),shape(1,2))
    mRobot.robotModel.plot(q);
end

function testDrawChar(mRobot, msg, scale)

    qz = [90 150]*pi/180;
    mRobot.moveSync(qz);

    char = msg(1,1);

    nameFile = ['Letters/letter',char,'.mat']
    load(nameFile,'letter');

    q = mRobot.robotModel.getpos();
    posX = q(1,1);
    posY = q(1,2);
    
    letter(:,1) = scale*letter(:,1) + posX;
    letter(:,2) = scale*letter(:,2) + posY;
    letter(:,3) = mRobot.PEN_DOWN;
    
    mRobot.robotModel.plot(letter(:,1:2),'fps',12);
    
    mRobot.penUpDown(mRobot.PEN_DOWN);
    
    % Plot Shape
    for i = 1:size(letter,1)
        q = mRobot.ikine(letter(i,1),letter(i,2));
        mRobot.moveFast(q);
        mRobot.penUpDown(letter(i,3));
    end
    
%     mRobot.penUpDown(mRobot.PEN_UP);

%     % Move to the next Letter
%     finalLetter = zeros(2);
%     finalLetter(1) = max(letter(:,1));
%     finalLetter(2) = min(letter(:,2));
%     q = mRobot.ikine(finalLetter(1,1),finalLetter(1,2));
%     mRobot.moveFast(q);
end

function testDrawWord(mRobot, msg, scale)
    for i = 1:size(msg,2);
        mRobot.drawLetter(msg(i),scale);
        mRobot.penUpDown(mRobot.PEN_UP);
    end
end

function testNormLetter(char)
    
    nameFile = ['Letters/letter',char,'.mat'];
    load(nameFile,'letter');

    nletter = letter;

    x1 = min(letter(:,1));x2=max(letter(:,1)); nletter(:,1) = (letter(:,1)-x1)/(x2 -x1);
    y1 = min(letter(:,2));y2=max(letter(:,2)); nletter(:,2) = (letter(:,2)-y1)/(y2 -y1);
    P = (x2 - x1)/(y2-y1);
    
    nletter(:,1) = nletter(:,1)*P;
    
    letter = nletter;
    nameFile = ['Letters/nletter',char,'.mat'];
    save(nameFile,'letter');
end

function testTransposeLetter(char)
    
    nameFile = ['Letters/letter',char,'.mat'];
    load(nameFile,'letter');

    nletter = letter;

    x1 = min(letter(:,2));x2=max(letter(:,2)); nletter(:,2) = (letter(:,2)-x1)/(x2 -x1);
    y1 = min(letter(:,1));y2=max(letter(:,1)); nletter(:,1) = (letter(:,1)-y1)/(y2 -y1);
    P = (x2 - x1)/(y2-y1);
    
    nletter(:,1) = nletter(:,1)*P;
    
    letter = nletter;
    nameFile = ['Letters/nletter',char,'.mat'];
    save(nameFile,'letter');
end

function testSizeLetter(mRobot)
    nchar = 3
    
    dx = 0;
    dy = 3;

    q = mRobot.robotModel.getpos();
    T = mRobot.robotModel.fkine(q);
    
    posx = T(1,4);
    posy = T(2,4);
    
    for i=1:nchar
        mRobot.penUpDown(mRobot.PEN_DOWN);
        posx = posx + dx;
        posy = posy + dy;
        newQ = mRobot.ikine(posx, posy)
        mRobot.penUpDown(mRobot.PEN_UP);
        pause(0.2)
        mRobot.moveFast(newQ)
    end
end

function testNormAll()
    abc = 'ABCDEFGHILMNOPQRSTUVWXYZ'
    
    for i = 1:size(abc,2)
        testNormLetter(abc(i));
    end
end

function testPlottingMap()
    map = makemap(20)
    
    ds = Dstar(map);
    figure;
    ds.plot();
end

function testDrawingPath(mRobot)
    % Create Map
    res = 20
    map = makemap(res)
    
    % Calculate Path base on this coordinates
    start = [2,18]
    goal = [18,2]
    
    ds = Dstar(map);
    ds.plan(goal);
    
    figure;
    path = ds.path(start);
    
    posX = 1;
    posY = 1;
    scale = 5/res
    
    q = mRobot.ikine(posX,posY);
    mRobot.penUpDown(mRobot.PEN_UP);
    mRobot.robotModel.plot(mRobot.robotPos);
    mRobot.moveFast(q);
    
    mRobot.robotToolkitDisp(['Drawing Path ',char]);
    
    path(:,1) = scale*path(:,1) + posX
    path(:,2) = scale*path(:,2) + posY
    
    mRobot.drawCurve(path);
end
