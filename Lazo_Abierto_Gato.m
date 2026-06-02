%Limpieza de pantalla
clear all
close all
clc

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=40;             % Tiempo de simulación en segundos (s)
ts=0.1;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras

%2 INTERPOLACIÓN DE TRAYECTORIA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Puntos = [-13.7,3.8; -13.7,10.7; -8.7,15.8; -5.1,15.8; -5.1,14.1; -8.6,10.7; -10.3,10.7; -10.3,5.5; -6.9,8.9;...
          -3.4,8.9; 0,5.5; 0,12.3; 3.5,9; 10.2,9; 13.7,12.3; 13.7,2.1; 11.8,0.5; 8.5,-1.2;...
          5.1,-1.2; 1.7,0.5; 0,2.1; 1.7,3.8; 1.7,6.9; 4.8,6.9; 4.8,3.8; 6.8,3.8; 8.9,3.8; ...
          8.9,6.9; 12,6.9; 12,3.8; 6.8,3.8; 8.9,2.1; 4.8,2.1; 6.8,3.8; 1.7,3.8; 0,2.1;  ...
          0,-3; 1.7,-3; 1.7,-6; -3.4,-6; -3.4,0; -6.9,0; -8.6,-1.2; -8.6,-3; -5.9,-3; -5.9,-6; -12,-6; -13.7,3.8];

s = linspace(0, 1, size(Puntos, 1));
ss = linspace(0, 1, N);
hx_ref = interp1(s, Puntos(:,1), ss, 'linear');
hy_ref = interp1(s, Puntos(:,2), ss, 'linear');

%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros(1, N+1); 
y1 = zeros(1, N+1); 
phi = zeros(1, N+1);
u = zeros(1, N);
w = zeros(1, N);

x1(1) = Puntos(1,1);    % Iniciamos en el punto A
y1(1) = Puntos(1,2);
phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1)); % Orientación inicial hacia el punto B

%4 BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:N-1
    % Derivadas para obtener velocidades deseadas
    dx = (hx_ref(k+1) - hx_ref(k))/ts;
    dy = (hy_ref(k+1) - hy_ref(k))/ts;
    
    u(k) = sqrt(dx^2 + dy^2); % Velocidad lineal
    
    % Calculamos la orientación deseada
    phi_d = atan2(dy, dx);
    
    % Velocidad angular 
    w(k) = atan2(sin(phi_d - phi(k)), cos(phi_d - phi(k)))/ts;
    
    % Actualizamos la orientación para el siguiente paso del cálculo
    phi(k+1) = phi(k) + w(k)*ts;
end

phi = zeros(1, N+1);
phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1));

for k=1:N 
    phi(k+1) = phi(k) + w(k)*ts; 
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    x1(k+1) = x1(k) + xp1*ts;
    y1(k+1) = y1(k) + yp1*ts;
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
H2=plot3(x1(1),y1(1),0,'r','lineWidth',2);
H3 = plot3(hx_ref(N), hy_ref(N), 0, 'bo', 'lineWidth', 2, 'MarkerSize', 10); 
H4 = plot3(hx_ref(1), hy_ref(1), 0, 'go', 'lineWidth', 2, 'MarkerSize', 10);

% d) Bucle de simulacion de movimiento del robot
step=1; % pasos para simulacion
for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx_ref(1:k),hy_ref(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(311)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
dist_error = sqrt((hx_ref - x1(1:N)).^2 + (hy_ref - y1(1:N)).^2);
plot(t, dist_error, 'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');

