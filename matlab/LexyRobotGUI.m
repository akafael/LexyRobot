function varargout = LexyRobotGUI(varargin)
%LEXYROBOTGUI M-file for LexyRobotGUI.fig
%      LEXYROBOTGUI, by itself, creates a new LEXYROBOTGUI or raises the existing
%      singleton*.
%
%      H = LEXYROBOTGUI returns the handle to a new LEXYROBOTGUI or the handle to
%      the existing singleton*.
%
%      LEXYROBOTGUI('Property','Value',...) creates a new LEXYROBOTGUI using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to LexyRobotGUI_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      LEXYROBOTGUI('CALLBACK') and LEXYROBOTGUI('CALLBACK',hObject,...) call the
%      local function named CALLBACK in LEXYROBOTGUI.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LexyRobotGUI

% Last Modified by GUIDE v2.5 11-Dec-2015 01:33:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LexyRobotGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @LexyRobotGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before LexyRobotGUI is made visible.
function LexyRobotGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for LexyRobotGUI
handles.output = hObject;

%% Run robotic toolkit
try
    startup_rvc;
catch ME
    errordlg('Robotic Toolkit not Find');
    rethrow ME;
end

%% Create Robot
handles.mRobot = lexyRobot();

%% Prepare GUI
handles.mRobot.isConnected2Arduino = false;     % Connection Status Flag
movegui(hObject,'center');                      % Change GUI position
axes(handles.axes1);                            % Chose Axis

serialPorts = instrhwinfo('serial');
nPorts = length(serialPorts.SerialPorts);
set(handles.menuportarduino, 'String', ...
    [{'Select a port'} ; serialPorts.SerialPorts ]);

% Map
handles.sizeMap = 40;
handles.map = [];

% Path Planning
handles.start = [0,0];
handles.goal = [0,0];


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LexyRobotGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LexyRobotGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in plot_graph.
function plot_graph_Callback(hObject, eventdata, handles)
% hObject    handle to plot_graph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get angles and convert to radians
handles.jointangle1 = (pi/180)*str2double(get(handles.angle1,'String'));
handles.jointangle2 = (pi/180)*str2double(get(handles.angle2,'String'));

newQ = [handles.jointangle1 handles.jointangle2];

axes(handles.axes1); 

try
    set(handles.textRobotModel,'String','Moving Joints');
    handles.mRobot.moveSync(newQ);
    set(handles.textRobotModel,'String','Done! =D');
catch ME
    errordlg('Unable to Move Robot');
    set(handles.textRobotModel,'String','Error =(');
end

statusRobot_update(hObject, eventdata, handles);

guidata(hObject, handles);

% --- Executes on button press in animation.
function animation_Callback(hObject, eventdata, handles)
% hObject    handle to animation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
angle1 = -pi/2:0.1:pi/2;
angle2 = angle1- pi;

newQ = [angle1' angle2'];

axes(handles.axes1);

for i = 1:size(newQ,1)

    handles.mRobot.robotModel.plot(newQ(i,:), 'fps', 10);
    
    statusRobot_update(hObject, eventdata, handles);
    
    pause(0.2);
end

% --- Executes on button press in btnConnectArduino.
function btnConnectArduino_Callback(hObject, eventdata, handles)
% hObject    handle to btnConnectArduino (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if (handles.mRobot.isConnected2Arduino)
	handles.mRobot.disconnectArduino();
	handles.mRobot.isConnected2Arduino = false;
	set(handles.btnConnectArduino,'String','Connect');
	set(handles.textStatusConnectionArduino,'String', 'Not connected.');
else
	try
        serPortn = get(handles.menuportarduino, 'Value');
        if serPortn == 1
            errordlg('Select valid COM port');
        else
            serList = get(handles.menuportarduino,'String');
            port = serList{serPortn};

            set(handles.textStatusConnectionArduino,'String', ['Connecting...']);
            % Connect
            handles.mRobot.robotArduino = handles.mRobot.connectArduino(port);
            handles.mRobot.isConnected2Arduino = true;
            
            % Update Gui
            set(handles.textStatusConnectionArduino,'String', ['Connected in ',port]);
            set(handles.btnConnectArduino,'String','Disconect');
            handles.mRobot.robotModel.plot(handles.mRobot.robotPos);
            statusRobot_update(hObject, eventdata, handles);
            set(handles.textRobotModel,'String', 'Simulation Ready!');
        end
	catch ME
		set(handles.textStatusConnectionArduino,'String', 'Connection error!');
		rethrow(ME);
	end
end

% Update handles structure
guidata(hObject, handles);

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.mRobot.isConnected2Arduino
    % Disconect Arduino
    try
      handles.mRobot.disconnectArduino();
    catch ME
      % rethrow ME
    end
end
% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on button press in btnCreateModel.
function btnCreateModel_Callback(hObject, eventdata, handles)
% hObject    handle to btnCreateModel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Get the Default Position and plot
    newQ = handles.mRobot.robotPos;
    handles.mRobot.robotModel.plot(newQ);
    movegui(handles.output,'center');
    
    statusRobot_update(hObject, eventdata, handles);
    
function editwrite_Callback(hObject, eventdata, handles)
% hObject    handle to editwrite (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editwrite as text
%        str2double(get(hObject,'String')) returns contents of editwrite as a double


% --- Executes during object creation, after setting all properties.
function editwrite_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editwrite (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function editposy_Callback(hObject, eventdata, handles)
% hObject    handle to editposy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editposy as text
%        str2double(get(hObject,'String')) returns contents of editposy as a double


% --- Executes during object creation, after setting all properties.
function editposy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editposy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editposx_Callback(hObject, eventdata, handles)
% hObject    handle to editposy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editposy as text
%        str2double(get(hObject,'String')) returns contents of editposy as a double


% --- Executes during object creation, after setting all properties.
function editposx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editposy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function angle2_Callback(hObject, eventdata, handles)
% hObject    handle to angle2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angle2 as text
%        str2double(get(hObject,'String')) returns contents of angle2 as a double


% --- Executes during object creation, after setting all properties.
function angle2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angle2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in menuportarduino.
function menuportarduino_Callback(hObject, eventdata, handles)
% hObject    handle to menuportarduino (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns menuportarduino contents as cell array
%        contents{get(hObject,'Value')} returns selected item from menuportarduino


% --- Executes during object creation, after setting all properties.
function menuportarduino_CreateFcn(hObject, eventdata, handles)
% hObject    handle to menuportarduino (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');	
end

% --- Executes on button press in btnChangePos.
function btnChangePos_Callback(hObject, eventdata, handles)
% hObject    handle to btnChangePos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get angles and convert to radians
handles.X = str2double(get(handles.editposx,'String'));
handles.Y = str2double(get(handles.editposy,'String'));

axes(handles.axes1); 

try
    set(handles.textRobotModel,'String',...
        ['Moving to (',num2str(handles.X),',',num2str(handles.Y),')']);
    newQ = handles.mRobot.ikine(handles.X, handles.Y);
    handles.mRobot.moveSync(newQ);
    statusRobot_update(hObject, eventdata, handles);
    set(handles.textRobotModel,'String', 'Done! =D');
catch ME
    errordlg('Unable to Move Robot');
    set(handles.textRobotModel,'String','Error =(');
    rethrow(ME);
end

% --- Custom Function to update the Status Painel after each moviment
function statusRobot_update(hObject, eventdata, handles)
% hObject    handle to btnChangePos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    newQ = handles.mRobot.robotModel.getpos();

    set(handles.angle1,'String',num2str(round(newQ(1)*180/pi)));
    set(handles.angle2,'String',num2str(round(newQ(2)*180/pi)));

   finalTransformation = handles.mRobot.robotModel.fkine(newQ);
    
    x = finalTransformation(1,4);
    y = finalTransformation(2,4);
    z = finalTransformation(3,4);
    
    set(handles.xpos,'String',num2str(x));
    set(handles.ypos,'String',num2str(y));
    set(handles.zpos,'String',num2str(z));
    
    set(handles.editposx,'String',num2str(x));
    set(handles.editposy,'String',num2str(y));
    
    finalOrientation = tr2rpy(finalTransformation, 'deg');
    
    roll = finalOrientation(1);
    pitch = finalOrientation(2);
    yaw = finalOrientation(3);
    
    set(handles.rollVal,'String',num2str(roll));
    set(handles.pitchVal,'String',num2str(pitch));
    set(handles.yawVal,'String',num2str(yaw));



function angle1_Callback(hObject, eventdata, handles)
% hObject    handle to angle1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angle1 as text
%        str2double(get(hObject,'String')) returns contents of angle1 as a double


% --- Executes during object creation, after setting all properties.
function angle1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angle1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnMovePen.
function btnMovePen_Callback(hObject, eventdata, handles)
% hObject    handle to btnMovePen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    if handles.mRobot.penState == handles.mRobot.PEN_UP
       handles.mRobot.penState = ...
           handles.mRobot.penUpDown(handles.mRobot.PEN_DOWN);
       set(handles.btnMovePen, 'String', 'Pen Up');
    else
       handles.mRobot.penState = handles.mRobot.penUpDown(handles.mRobot.PEN_UP);
        set(handles.btnMovePen, 'String', 'Pen Down');
    end
    
    % Update handles structure
    guidata(hObject, handles);


% --- Executes on button press in btnWrite.
function btnWrite_Callback(hObject, eventdata, handles)
% hObject    handle to btnWrite (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    % Get Message to write
    msg = get(handles.editwrite,'String');
    msg = upper(msg);
    
    set(handles.textRobotModel,'String', 'Drawing message.');

    for i = 1:size(msg,2);
        handles.mRobot.plotLetter(msg(i),handles.axesLetter);
        handles.mRobot.drawLetter(msg(i),1);
        handles.mRobot.penUpDown(handles.mRobot.PEN_UP);
        statusRobot_update(hObject, eventdata, handles);
    end
    
    % Update GUI
    set(handles.textRobotModel,'String', 'Done!');
    set(handles.editwrite,'String','');


% --- Executes on button press in btnWriteName.
function btnWriteName_Callback(hObject, eventdata, handles)
% hObject    handle to btnWriteName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    set(handles.textRobotModel,'String', 'Preparing...');

    % Go for the left side of the page        
    handles.X = -1;
    handles.Y = 4;
    
    handles.mRobot.penUpDown(handles.mRobot.PEN_UP);
    newQ = handles.mRobot.ikine(handles.X, handles.Y);
    handles.mRobot.moveSync(newQ);
    
    statusRobot_update(hObject, eventdata, handles);

    % Get Robot name
    msg = handles.mRobot.name;
    msg = upper(msg);
    
    % Write Robot Name
    set(handles.textRobotModel,'String', 'Writing robot name');
    for i = 1:size(msg,2);
        handles.mRobot.plotLetter(msg(i),handles.axesLetter);
        handles.mRobot.drawLetter(msg(i),1);
    end
    
    % Update Status
    statusRobot_update(hObject, eventdata, handles);
    set(handles.textRobotModel,'String', 'Done');


% --- Executes during object creation, after setting all properties.
function axesLetter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axesLetter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axesLetter
	% hide axis
    % axis(hObject,'off');


% --- Executes on button press in btnPath.
function btnPath_Callback(hObject, eventdata, handles)
% hObject    handle to btnPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try
    figure;
    handles.pathmap.plot();
catch ME
    errordlg('Map Not find!');
end

% --- Executes on button press in btnMap.
function btnMap_Callback(hObject, eventdata, handles)
% hObject    handle to btnMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% get current figure
fig = gca;

% Show help msg
% msgbox({'left button, click and drag to create a rectangle';...
%   'or type the following letters in the figure window:';...
%   'p - draw polygon';...
%   'c - draw circle';...
%   'e - erase map';...
%   'u - undo last action';...
%   'q - leave editing mode'},'makemap');

% Open Map editor
mapFig = figure;
handles.map = makemap(handles.sizeMap);
uiwait(mapFig);

% Create a Basic Map
handles.pathmap = PRM(handles.map);

% Select previews Axis
axes(fig);

% Status message
set(handles.textRobotModel,'String', 'Map Edited!');

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in btnDstart.
function btnSetStartGoal_Callback(hObject, eventdata, handles)
% hObject    handle to btnDstart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set Start Point
startX = str2num(get(handles.editStartX,'String'));
startY = str2num(get(handles.editStartY,'String'));
handles.start = [startX,startY];

% Set Goal Point
goalX = str2num(get(handles.editGoalX,'String'));
goalY = str2num(get(handles.editGoalY,'String'));
handles.goal = [goalX,goalY];

set(handles.textRobotModel,'String', 'Start and Goal Defined!');

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in btnDstart.
function btnDstart_Callback(hObject, eventdata, handles)
% hObject    handle to btnDstart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Calculate Path D*
ds = Dstar(handles.map);
ds.plan(handles.goal);
figure;
ds.path(handles.start);
handles.ds = ds;

handles.pathmap = handles.ds;

set(handles.textRobotModel,'String', 'D* Path Created!');

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in btnPRM.
function btnPRM_Callback(hObject, eventdata, handles)
% hObject    handle to btnPRM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Calculate Path PRM
prm = PRM(handles.map);
prm.plan();
figure;
prm.path(handles.start,handles.goal);
handles.prm = prm;

set(handles.textRobotModel,'String', 'PRM Path Created!');

handles.pathmap = handles.prm;

% Update handles structure
guidata(hObject, handles);


function editStartX_Callback(hObject, eventdata, handles)
% hObject    handle to editStartX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStartX as text
%        str2double(get(hObject,'String')) returns contents of editStartX as a double


% --- Executes during object creation, after setting all properties.
function editStartX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStartX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editStartY_Callback(hObject, eventdata, handles)
% hObject    handle to editStartY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStartY as text
%        str2double(get(hObject,'String')) returns contents of editStartY as a double


% --- Executes during object creation, after setting all properties.
function editStartY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStartY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editGoalX_Callback(hObject, eventdata, handles)
% hObject    handle to editGoalX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editGoalX as text
%        str2double(get(hObject,'String')) returns contents of editGoalX as a double


% --- Executes during object creation, after setting all properties.
function editGoalX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editGoalX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editGoalY_Callback(hObject, eventdata, handles)
% hObject    handle to editGoalY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editGoalY as text
%        str2double(get(hObject,'String')) returns contents of editGoalY as a double


% --- Executes during object creation, after setting all properties.
function editGoalY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editGoalY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in btnRunPath.
function btnRunPath_Callback(hObject, eventdata, handles)
% hObject    handle to btnRunPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    set(handles.textRobotModel,'String', 'Preparing...');

    handles.X = -1;
    handles.Y = 4;
    
    handles.mRobot.penUpDown(handles.mRobot.PEN_UP);
    newQ = handles.mRobot.ikine(handles.X, handles.Y);
    handles.mRobot.moveSync(newQ);
    
    statusRobot_update(hObject, eventdata, handles);
    
    % Generate Path Curve file
    
    % Draw Path Curve
    
    
    % Update Status
    statusRobot_update(hObject, eventdata, handles);
    set(handles.textRobotModel,'String', 'Done');
