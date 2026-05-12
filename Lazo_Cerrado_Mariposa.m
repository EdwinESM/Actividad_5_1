clear; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTOS DE GEOGEBRA %%%%%%%%%%%%%%%%%%%%%%%%%%%%
Puntos = [3,6; 3,7; 4,7.5; 3.5,7.8; 3,8.5; 3.4,9.5; 4.3,9.5; 5,9.2; ...
          5.5,9.7; 6,10; 5.5,9.7; 5,10; 5.5,9.7; 6,9.2; 6.6,9.5; 7.6,9.4; 8,8.5; ...
          7.5,7.8; 7,7.5; 8,7; 8,6; 7,5.5; 6.5,6; 6,6.5; 6,9.2; 6,6.5; ...
          5.5,6; 5,6.5; 5,9.2; 5,6.5; 4.5,6 ; 4,5.5; 3,6];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 100;   
ts = 0.1; 
t = 0: ts: tf; 
N = length(t);

%%%%%%%%%%%%%%%%%%%%%%%% GENERACIÓN DE TRAYECTORIA TEMPORAL %%%%%%%%%%%%%%%
% Creamos la referencia x(t) y y(t) para que recorra todos los puntos en tf
distancias = [0; cumsum(sqrt(sum(diff(Puntos).^2, 2)))]; % Distancia acumulada
t_ref = linspace(0, tf, size(Puntos,1)); % Tiempo asignado a cada punto
% Referencia suave o lineal 
hx_ref = interp1(t_ref, Puntos(:,1), t, 'linear');
hy_ref = interp1(t_ref, Puntos(:,2), t, 'linear');

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1, N+1); y1 = zeros(1, N+1); phi = zeros(1, N+1); 
x1(1) = 3; y1(1) = 6; phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1));

% Parámetros del controlador
K = 4.0; 

u = zeros(1, N); w = zeros(1, N);

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE CONTROL (TRACKING) %%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % El punto objetivo depende del tiempo
    xref = hx_ref(k);
    yref = hy_ref(k);
    
    % Calcular error de posición 
    ex = xref - x1(k);
    ey = yref - y1(k);
    
    % Transforma el error al marco del robot 
   
    vx = K * ex;
    vy = K * ey;
    
    % 4. Convertir velocidades globales a velocidades del robot (u, w)
    u(k) = vx * cos(phi(k)) + vy * sin(phi(k));
    w(k) = (1/0.1) * (vy * cos(phi(k)) - vx * sin(phi(k))); 
    
    % Limitación de seguridad
    u(k) = clip(u(k), -1.5, 1.5);
    w(k) = clip(w(k), -4, 4);
    
    % 5. Modelo cinemático
    phi(k+1) = phi(k) + w(k)*ts;
    x1(k+1) = x1(k) + u(k)*cos(phi(k+1))*ts;
    y1(k+1) = y1(k) + u(k)*sin(phi(k+1))*ts;
end

% Función auxiliar para limitar velocidad
function out = clip(val, low, high)
    out = max(min(val, high), low);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene=figure;  
set(scene,'Color','white'); 
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize'); 
set(scene,'position',sizeScreen); 
camlight('headlight'); 
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
view([0 90]); 
axis([2 9 4 11]); 

% Graficar robots en la posicion inicial
scale = 1.5; 
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2=plot3(x1(1),y1(1),0,'r','lineWidth',2);

% Bucle de movimiento
step=2; 
for k=1:step:N
    delete(H1);    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    set(H2, 'XData', x1(1:k), 'YData', y1(1:k), 'ZData', zeros(1,k));
    pause(0.01); 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas de Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  
set(graph,'position',sizeScreen); 
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('Lineal [m/s]'),legend('u');
title('Velocidades de Referencia para la Mariposa');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('Angular [rad/s]'),legend('w');