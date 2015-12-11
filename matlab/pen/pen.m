function pen
% Pen-Like data processing template
% pen.m is a GUI ready to use
%       the GUI calls a function called "process_data"
%       located at the end of the file every time the pen passes over the red area
%       include your processing code there... ! 
%
% (c) 2009 Grup de Robotica de la UdL
% http://robotica.udl.es
global estat_pulsacio

himatge = findobj('tag','PEN');
if (isempty(himatge))
    estat_pulsacio = 0;

    % crear la nueva figura
    himatge = figure; % es genera una figura i li assigna un identificador
	
	hvisio.versio = '0.01';

    set(himatge,'numbertitle','off');               % treu el numero de figura
    set(himatge,'name',['PEN (' hvisio.versio ')']);    % nom
    set(himatge,'MenuBar','none');                  % treiem el menu d'icons
    set(himatge,'doublebuffer','on');               % dos buffers de grafics
    set(himatge,'tag','PEN');                       % identifiquem la figura
    set(himatge,'Color',[0.95 0.95 0.95]);
    set(himatge,'WindowButtonMotionFcn',@moviment);
    set(himatge,'WindowButtonDownFcn',@moviment_down);
    set(himatge,'WindowButtonUpFcn',@moviment_up);
       
    % crear l'axis
    h_axes = axes('position', [0 0 1 0.85]);
    set(h_axes,'Tag','AXES');
    box(h_axes,'on');
    grid(h_axes,'on');
    axis(h_axes,[0 1 0 1]);
    axis(h_axes,'off');
    hold(h_axes,'on');
    
    % crear les marques de borrat
    fill([0 0.1 0.1 0 0],[0 0 0.1 0.1 0],'r');
    fill([0.9 1 1 0.9 0.9],[0 0 0.1 0.1 0],'r');
    
    % crear el text
    h_text = uicontrol('Style','edit','Units','normalized','Position',[0 0.85 1 0.15],'FontSize',14,'HorizontalAlignment','left','Enable','inactive','Tag','TEXT');
       
	% ######  MENU  ######################################
    h_opt = uimenu('Label','&Options');
        uimenu(h_opt,'Label','Manual mode {input processed at the red area}','Enable','off');
        uimenu(h_opt,'Label','Clear / Restore','Callback',@mostrar);
        h_ref = uimenu(h_opt,'Label','References','separator','on');
        uimenu(h_ref,'Label','Rectangular','Callback',@referencies,'Tag','NORMAL','UserData','REF');
        uimenu(h_ref,'Label','Italic','Callback',@referencies,'Tag','ITALIC','UserData','REF');
        uimenu(h_ref,'Label','None','Callback',@referencies,'Tag','CAP','UserData','REF','Checked','on');
    
        uimenu(h_opt,'Label','Sortir - Exit','Callback','closereq;','separator','on');
        
    h_opt2 = uimenu('Label','&About pen.m');
        uimenu(h_opt2,'Label','Grup de Robotica de la Universitat de Lleida');
        uimenu(h_opt2,'Label','Grupo de Robotica de la Universidad de Lleida');
        uimenu(h_opt2,'Label','Robotic Team, University of Lleida (Spain)');
        uimenu(h_opt2,'Label','Browse -> http://robotica.udl.cat','Callback','web http://robotica.udl.cat -browser','separator','on');
        
else
    figure(himatge);
end
% #########################################################################

% #########################################################################
function mostrar(hco,eventStruct)
% seleccionar mode
global x_pen y_pen

% borrar dibuix anterior
delete(findobj('Tag','TRAJECTORIA'));
delete(findobj('Tag','TRAJECTORIA2'));

% borrar dades anteriors
x_pen = [];
y_pen = [];

% per si fa falta
himatge = findobj('tag','PEN');

set(himatge,'WindowButtonMotionFcn',@moviment);
set(himatge,'WindowButtonDownFcn',@moviment_down);
set(himatge,'WindowButtonUpFcn',@moviment_up);
% #########################################################################

% #########################################################################
function referencies(hco,eventStruct)
% marcar referencies

set(findobj('UserData','REF'),'Checked','off');
set(findobj('Tag',get(hco,'Tag')),'Checked','on');

% borrar anteriors
delete(findobj('Tag','REFERENCIES'));

% dibuixar les noves
h_axes = findobj('Tag','AXES');

switch get(hco,'Tag')
case {'NORMAL'}
    for i = 0:0.1:1
        % horitzontal
        plot(h_axes,[0 1],[i i],'k:','Color',[0.5 0.5 0.5],'Tag','REFERENCIES');
        
        % vertical
        plot(h_axes,[i i],[0 1],'k:','Color',[0.5 0.5 0.5],'Tag','REFERENCIES');
    end
    
case {'ITALIC'}
    for i = 0:0.1:1
        % horitzontal
        plot(h_axes,[0 1],[i i],'k:','Color',[0.5 0.5 0.5],'Tag','REFERENCIES');
        
        % vertical
        plot(h_axes,[i i+0.1],[0 1],'k:','Color',[0.5 0.5 0.5],'Tag','REFERENCIES');
    end
end
% #########################################################################

% #########################################################################
function moviment_down(hco,eventStruct)
% assegurar que no hi ha cap captura en marxa
global estat_pulsacio x_pen y_pen

% toggle
estat_pulsacio = 1;

% recuperar punt
h_axes = findobj('Tag','AXES');
p = get(h_axes,'CurrentPoint');
x = p(1,1);
y = p(1,2);

% dades acumulatives
x_pen = [x_pen x];
y_pen = [y_pen y];

% dibuixar
plot(h_axes,x,y,'b.','Tag','TRAJECTORIA');
% #########################################################################

% #########################################################################
function moviment_up(hco,eventStruct)
% assegurar que no hi ha cap captura en marxa
global estat_pulsacio x_pen y_pen

% toggle
estat_pulsacio = 0;

h_axes = findobj('Tag','AXES');

% analisi del que s'ha polsat
% borrar requadre anterior
delete(findobj('Tag','TRAJECTORIA2'));

% marcar un requadre
x_i = min(x_pen);
x_f = max(x_pen);
x_d = max([1 (x_f - x_i)]);
y_i = min(y_pen);
y_f = max(y_pen);
y_d = max([1 (y_f - y_i)]);
plot(h_axes,[x_i x_f x_f x_i x_i],[y_i y_i y_f y_f y_i],'K:','MarkerSize',22,'Tag','TRAJECTORIA2');
% #########################################################################

% #########################################################################
function moviment(hco,eventStruct)
% asegurar que no hi ha cap captura en marxa
global estat_pulsacio x_pen y_pen

h_axes = findobj('Tag','AXES');

p = get(h_axes,'CurrentPoint');
x = p(1,1);
y = p(1,2);

if estat_pulsacio
    % boto pulsat
    
    if ((y < 0) || (y > 1) || (x < 0) || (x > 1))
        % no fer res
        return;
    end
    
    if ((x ~= x_pen(end)) || (y ~= y_pen(end)))
        % punt seguent
        x_pen = [x_pen x];
        y_pen = [y_pen y];

        plot(h_axes,[x_pen(end-1) x],[y_pen(end-1) y],'b.-','Tag','TRAJECTORIA');
    end
   
else
    
    if ((x < 0.1) && (y < 0.1)) || ((x > 0.9) && (y < 0.1))
        % al passar sobre la zona roja
        if (~isempty(x_pen))
            
            % processar la paraula
            process_data(x_pen,y_pen);
           
            % borrar dibuix
            delete(findobj('Tag','TRAJECTORIA'));
            delete(findobj('Tag','TRAJECTORIA2'));

            % borrar dades
            x_pen = [];
            y_pen = [];
        end
    end   
end
% #########################################################################

% #########################################################################
function process_data(x_pen,y_pen)
persistent num_blocks

if isempty(num_blocks)
    num_blocks = 1;
else
    num_blocks = num_blocks + 1;
end

% your code for word recognition / draw processing starts here
% x_pen, y_pen are the current point locations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


set(findobj('Tag','TEXT'),'String',[' ' num2str(num_blocks) ' inputs processed']);
% #########################################################################
