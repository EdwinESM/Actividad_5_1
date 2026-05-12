clear; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTOS DE GEOGEBRA %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definimos la lista de puntos según el orden proporcionado
Puntos = [3,6; 3,7; 4,7.5; 3.5,7.8; 3,8.5; 3.4,9.5; 4.3,9.5; 5,9.2; ...
          5.5,9.7; 6,10; 5.5,9.7; 5,10; 5.5,9.7; 6,9.2; 6.6,9.5; 7.6,9.4; 8,8.5; ...
          7.5,7.8; 7,7.5; 8,7; 8,6; 7,5.5; 6.5,6; 6,6.5; 6,9.2; 6,6.5; ...
          5.5,6; 5,6.5; 5,9.2; 5,6.5; 4.5,6 ; 4,5.5; 3,6];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 60;                    % Aumentamos el tiempo para recorrer toda la figura
ts = 0.1;                   % Tiempo de muestreo
t = 0: ts: tf;              % Vector de tiempo
N = length(t);              % Muestras

%%%%%%%%%%%%%%%%%%%%%%%% INTERPOLACIÓN DE TRAYECTORIA %%%%%%%%%%%%%%%%%%%%%
% Creamos una trayectoria suave que pase por todos los puntos
s = linspace(0, 1, size(Puntos, 1));
ss = linspace(0, 1, N);
hx_ref = interp1(s, Puntos(:,1), ss, 'linear');
hy_ref = interp1(s, Puntos(:,2), ss, 'linear');

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1, N+1); 
y1 = zeros(1, N+1); 
phi = zeros(1, N+1); 

x1(1) = Puntos(1,1);    % Iniciamos en el punto A
y1(1) = Puntos(1,2);
phi(1) = atan2(hy_ref(2)-hy_ref(1), hx_ref(2)-hx_ref(1)); % Orientación inicial hacia el punto B

%%%%%%%%%%%%%%%%%%%%%% CÁLCULO DE VELOCIDADES (OPEN LOOP) %%%%%%%%%%%%%%%%%%
u = zeros(1, N);
w = zeros(1, N);

for k = 1:N-1
    % Derivadas para obtener velocidades deseadas
    dx = (hx_ref(k+1) - hx_ref(k))/ts;
    dy = (hy_ref(k+1) - hy_ref(k))/ts;
    
    u(k) = sqrt(dx^2 + dy^2); % Velocidad lineal
    
    % Calculamos la orientación deseada
    phi_d = atan2(dy, dx);
    
    % Velocidad angular (diferencia de ángulos)
    w(k) = atan2(sin(phi_d - phi(k)), cos(phi_d - phi(k)))/ts;
    
    % Actualizamos la orientación para el siguiente paso del cálculo
    phi(k+1) = phi(k) + w(k)*ts;
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reiniciamos phi para la simulación cinemática real
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
scene=figure;  
set(scene,'Color','white'); 
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize'); 
set(scene,'position',sizeScreen); 
camlight('headlight'); 
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
view([0 90]); % Vista superior para ver mejor la mariposa
axis([2 9 4 11]); 

% Graficar robots en la posicion inicial
scale = 1.5; % Escala ajustada para el tamaño de la mariposa
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2=plot3(x1(1),y1(1),0,'r','lineWidth',2);

% Bucle de movimiento
step=2; 
for k=1:step:N
    delete(H1);    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    set(H2, 'XData', x1(1:k), 'YData', y1(1:k), 'ZData', zeros(1,k));
    pause(0.01); % Pausa más corta para fluidez
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas de Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  
set(graph,'position',sizeScreen); 
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('Lineal [m/s]'),legend('u');
title('Velocidades de Referencia para la Mariposa');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('Angular [rad/s]'),legend('w');