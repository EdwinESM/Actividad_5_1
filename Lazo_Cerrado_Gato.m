%Limpieza de pantalla
clear all
close all
clc

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=80;             % Tiempo de simulación en segundos (s)
ts=0.1;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras

v = zeros(1, N);
w = zeros(1, N);
x1 = zeros(1, N+1);
y1 = zeros(1, N+1);
phi = zeros(1, N+1);
hx = zeros(1, N+1);
hy = zeros(1, N+1);
Error = zeros(1, N);

% 2 INTERPOLACIÓN DE TRAYECTORIA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Puntos = [-13.7,3.8; -13.7,10.7; -8.7,15.8; -5.1,15.8; -5.1,14.1; -8.6,10.7; -10.3,10.7; -10.3,5.5; -6.9,8.9;...
          -3.4,8.9; 0,5.5; 0,12.3; 3.5,9; 10.2,9; 13.7,12.3; 13.7,2.1; 11.8,0.5; 8.5,-1.2;...
          5.1,-1.2; 1.7,0.5; 0,2.1; 1.7,3.8; 1.7,6.9; 4.8,6.9; 4.8,3.8; 6.8,3.8; 8.9,3.8; ...
          8.9,6.9; 12,6.9; 12,3.8; 6.8,3.8; 8.9,2.1; 4.8,2.1; 6.8,3.8; 1.7,3.8; 0,2.1;  ...
          0,-3; 1.7,-3; 1.7,-6; -3.4,-6; -3.4,0; -6.9,0; -8.6,-1.2; -8.6,-3; -5.9,-3; -5.9,-6; -12,-6; -13.7,3.8];

s = linspace(0, 1, size(Puntos, 1));
ss = linspace(0, 1, N);
hx_ref = interp1(s, Puntos(:,1), ss, 'linear');
hy_ref = interp1(s, Puntos(:,2), ss, 'linear');

%3 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Iniciamos en el primer punto de la trayectoria
x1(1) = hx_ref(1);  
y1(1) = hy_ref(1);  
phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1)); 

% Inicialización de vectores para el punto de control
hx(1) = x1(1);       
hy(1) = y1(1);

%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 

    hxd = hx_ref(k);
    hyd = hy_ref(k);

    %a)Errores de control
    hxe(k)=hxd-hx(k);
    hye(k)=hyd-hy(k);
    
    %Matriz de error
    he= [hxe(k);hye(k)];
    %Magnitud del error de posición
    Error(k)= sqrt(hxe(k)^2 +hye(k)^2);

    %b)Matriz Jacobiana
    J = [cos(phi(k)) -sin(phi(k));... 
         sin(phi(k))  cos(phi(k))];

    %c)Matriz de Ganancias
    K = [2.5 0;...
         0 2.25];

    %d)Ley de Control
    qpRef= pinv(J)*K*he;

    v(k)= qpRef(1);   %Velocidad lineal de entrada al robot 
    w(k)= qpRef(2);   %Velocidad angular de entrada al robot 


%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Aplico la integral a la velocidad angular para obtener el angulo "phi" de la orientación
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
           
   %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    
    xp1=v(k)*cos(phi(k)); 
    yp1=v(k)*sin(phi(k));
 
    %Aplico la integral a la velocidad lineal para obtener las cordenadas
    %"x1" y "y1" de la posición
    x1(k+1)=x1(k)+ xp1*ts; % Integral numérica (método de Euler)
    y1(k+1)=y1(k)+ yp1*ts; % Integral numérica (método de Euler)

    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Configurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([-0.1 35]); % Orientacion de la figura
axis([-16 16 -8 18 0 1]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 2;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3 = plot3(hx_ref(N), hy_ref(N), 0, 'bo', 'lineWidth', 2, 'MarkerSize', 10); 
H4 = plot3(hx_ref(1), hy_ref(1), 0, 'go', 'lineWidth', 2, 'MarkerSize', 10);

% d) Bucle de simulacion de movimiento del robot
step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(311)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');

