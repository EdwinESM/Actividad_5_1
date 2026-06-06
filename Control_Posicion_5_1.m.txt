%Limpieza de pantalla
clear all
close all
clc

% TIEMPO
tf=15;
ts=0.1;
t=0:ts:tf;
N= length(t);

%% PUNTOS DESEADOS 
puntos = [1,2; 3,7; 6,0; -4,5; -6,0; -1,0; -7,-7; -2,-4; ...
          -0.5,-0.5; 1,-3; 3,-5; 8,0; 0,-3; 0,9; 0,-1; ...
          -5,-10; 7,-7; 3,-1; -10,-10; 10,9];

%% GANANCIAS A COMPARAR
ganancias = [0.5, 1.0, 2.0];

for p = 1:size(puntos,1)
  for g = 1:length(ganancias)

    % CONDICIONES INICIALES
    x1(1)=0;  y1(1)=0;  phi(1)=pi/2;

    % POSICION DESEADA
    hxd = puntos(p,1);
    hyd = puntos(p,2);
    hx(1)= x1(1);
    hy(1)= y1(1);

    % BUCLE DE CONTROL
    for k=1:N
      hxe(k)=hxd-hx(k);
      hye(k)=hyd-hy(k);
      he= [hxe(k); hye(k)];
      Error(k)= sqrt(hxe(k)^2 + hye(k)^2);

      J=[cos(phi(k)) -sin(phi(k));
         sin(phi(k))  cos(phi(k))];

      K_gain = ganancias(g) * eye(2);

      qpRef= pinv(J)*K_gain*he;
      v(k)= qpRef(1);
      w(k)= qpRef(2);

      phi(k+1)=phi(k)+w(k)*ts;
      xp1=v(k)*cos(phi(k));
      yp1=v(k)*sin(phi(k));
      x1(k+1)=x1(k)+ xp1*ts;
      y1(k+1)=y1(k)+ yp1*ts;
      hx(k+1)=x1(k+1);
      hy(k+1)=y1(k+1);
    end

    %GRAFICAS
    fig_idx = (p-1)*3 + g;
    figure(fig_idx);
    sgtitle(sprintf('Punto %d (%.1f, %.1f)  |  K = %.1f', p, hxd, hyd, ganancias(g)));

    subplot(311)
    plot(t,v,'b','LineWidth',2)
    grid on; xlabel('Tiempo [s]'); ylabel('m/s'); legend('Velocidad Lineal (v)');

    subplot(312)
    plot(t,w,'g','LineWidth',2)
    grid on; xlabel('Tiempo [s]'); ylabel('rad/s'); legend('Velocidad Angular (w)');

    subplot(313)
    plot(t,Error,'r','LineWidth',2)
    grid on; xlabel('Tiempo [s]'); ylabel('m'); legend('Error de posición');

  end 
end    