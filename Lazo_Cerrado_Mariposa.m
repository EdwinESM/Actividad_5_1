clear; close all; clc;

%%%%%%%%%%%%%%%%%%%%% PUNTOS DE GEOGEBRA %%%%%%%%%%%%%%%%%%%%%%%%%%%
Puntos = [3,6; 3,7; 4,7.5; 3.5,7.8; 3,8.5; 3.4,9.5; 4.3,9.5; 5,9.2; ...
          5.5,9.7; 6,10; 5.5,9.7; 5,10; 5.5,9.7; 6,9.2; 6.6,9.5; 7.6,9.4; 8,8.5; ...
          7.5,7.8; 7,7.5; 8,7; 8,6; 7,5.5; 6.5,6; 6,6.5; 6,9.2; 6,6.5; ...
          5.5,6; 5,6.5; 5,9.2; 5,6.5; 4.5,6 ; 4,5.5; 3,6];

%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 120; 
ts = 0.1; 
t = 0: ts: tf; 
N = length(t);

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1, N+1); y1 = zeros(1, N+1); phi = zeros(1, N+1); 
x1(1) = 3; y1(1) = 6; phi(1) = 0;

% PARAMETROS DEL CONTROLADOR
K = [0.8 0; 0 0.8];      % Matriz de Ganancia Proporcional
dist_tolerance = 0.07;   % Radio de llegada al punto 
target_idx = 1;          % Índice del punto actual al que vamos
a = 0.12;                % Offset 

u = zeros(1, N); w = zeros(1, N);
Error = zeros(1, N);

%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%
for k = 1:N 
    if target_idx <= size(Puntos, 1)
        % 1. Definir objetivo actual 
        hxd = Puntos(target_idx, 1);
        hyd = Puntos(target_idx, 2);
        
        % 2. Matriz de error 
        hxe = hxd - x1(k);
        hye = hyd - y1(k);
        he = [hxe; hye];
        dist = sqrt(hxe^2 + hye^2);
        Error(k) = dist;

        % 3. Matriz Jacobiana de Rotación con Offset (J)
        J = [cos(phi(k)), -a*sin(phi(k)); ...
             sin(phi(k)),  a*cos(phi(k))];

        % 4. Ley de Control Proporcional Matricial
        u_w = J \ (K * he); 
        
        u(k) = u_w(1);
        w(k) = u_w(2);

        % 5. Logica de cambio de punto
        if dist < dist_tolerance
            target_idx = target_idx + 1;
        end
    else
        % Si termino todos los puntos, se detiene
        u(k) = 0; w(k) = 0;
    end

    % Limitacion de seguridad para motores
    u(k) = max(min(u(k), 1.0), -1.0);
    w(k) = max(min(w(k), 2.5), -2.5);
    
    % 6. Modelo cinematico 
    phi(k+1) = phi(k) + w(k)*ts;
    x1(k+1) = x1(k) + u(k)*cos(phi(k+1))*ts;
    y1(k+1) = y1(k) + u(k)*sin(phi(k+1))*ts;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACIÓN VIRTUAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure; set(scene,'Color','white'); 
sizeScreen=get(0,'ScreenSize'); set(scene,'position',sizeScreen);
axis equal; grid on; view([0 90]); axis([2 9 4 11]); 
xlabel('x(m)'); ylabel('y(m)'); title('P2P Control - Mariposa');

scale = 1.2; MobileRobot_5;
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2 = plot(x1(1),y1(1),'r','lineWidth',2);
plot(Puntos(:,1), Puntos(:,2), 'bo', 'MarkerSize', 4); 

for k = 1:5:N
    if ishandle(H1), delete(H1); end
    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    set(H2, 'XData', x1(1:k), 'YData', y1(1:k));
    pause(0.001);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRÁFICAS DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure; 
set(graph,'position',sizeScreen,'Color','white');

subplot(3,1,1)
plot(t, u, 'b', 'LineWidth', 2); grid on;
ylabel('u [m/s]'); title('Velocidad Lineal');

subplot(3,1,2)
plot(t, w, 'r', 'LineWidth', 2); grid on;
ylabel('w [rad/s]'); title('Velocidad Angular');

subplot(3,1,3)
plot(t, Error, 'g', 'LineWidth', 2); grid on;
ylabel('Error [m]'); xlabel('Tiempo [s]'); title('Magnitud del Error de Posición');
