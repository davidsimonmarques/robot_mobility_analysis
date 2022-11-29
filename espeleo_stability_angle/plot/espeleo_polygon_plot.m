%_________________________________________  
%Diagrama da posicao dos pontos de contato do espeleo_robo:
%    (p6) ____(p1)             
%     |     X   |         
%  (p5)  y__|   (p2)       
%     |         |            
%   (p4) ____ (p3)             
%


%PSEUDOCODIGO:____________________________
%Entradas: m, q, p_R,fg
%Saidas: Y, alpha
%
%L1: rot_matrix = quat2rotm(q) --- Converte q em matriz de rotacao e guarda em R
%L2: p_I(i) = rot_matrix p_R(i) --- Calcula posicao dos pontos de contato de acordo
%com a orientacao do robo
%L3: f^g = fg/norm(fg) --- Normaliza fg
%L4: for i= 1,i=m,i=i+1 do
%L5:    a_t(i) = p_I_(i+1)_l -p_I_(i) --- Cálculo dos eixos de tombamento (arestas
%do poligono)
%L6:    a^_t(i) = a_t(i)/norm(a_t(i)) --- Normalizacao de a_t
%L7:    l(i) = (I - a^_t(i)transpose(a^_t(i))) p_I_(i+1) --- Calculo de l(i) normal a a_t(i)
%L8:    l^(i)= l(i)/norm(l(i)) --- Normalizacao de l
%--- Definicao do sinal do i-esimo angulo de tombamento
%L9:    if(l^(i) \times f^g)\dot at(i)> 0 then 
%L10:        sigma = 1
%L11:   else
%L12:       sigma = -1
%L13:   Y(i) = sigma acos(f^g \dot l^(i)) --- Calculo do i-esimo angulo de tombamento
%que e aquele entre o vetor da forca gravitacional fg e a normal l(i). 
%L14: end for
%L15: alpha = min(Y) --- Obtem do angulo minimo de Y
%L16: plot3d(p_I_i, a_t, l, alpha) --- Plota os pontos, os eixos de tombamento,
%os vetores normais aos eixos e o menor dos angulos.
%_________________________________________

clc;
%Parâmetros do espeleo_robo:---------------------------------------------
m = 6; %numero de pontos de contato                               |
dx = 0.212; %distancia entre p1 e p2 = distancia entre p2 e p3 ...|
dy = 0.33; %distancia entre p6 e p1 = distancia entre p4 e p3     |     
dy_m = 0.425; %distancia entre p5 e p2 - rodas mais afastadas     | 
dz = 0.135; %altura do centro de massa                            | 
o = [0; 0; 0];% definicao da origem (posicao do centro de massa)
fg = [0; 0; -1]; %vetor gravidade
%Localizacao dos pontos de contato com o terreno de acordo com o sistema de
%coordenadas do robo:---------------------------------------------
p_R_1 = [ dx+o(1); -(dy/2)+o(2); -dz+o(3)]; 
p_R_2 = [  o(1); -(dy_m/2)+o(2); -dz+o(3)]; 
p_R_3 = [-dx+o(1); -(dy/2)+o(2); -dz+o(3)];  
p_R_4 = [-dx+o(1);  (dy/2)+o(2); -dz+o(3)]; 
p_R_5 = [  o(1);  (dy_m/2)+o(2); -dz+o(3)];
p_R_6 = [ dx+o(1);  (dy/2)+o(2); -dz+o(3)];
%Inicializacoes:---------------------------------------------
a = zeros(m,3); %matriz dos eixos de tombamento
I = eye(3,3); %Matriz Identidade
l = zeros(m,3);
%...................::::ROS::::..............................
%sub = rossubscriber('/imu/data', 'sensor_msgs/Imu');
sub = rossubscriber('/phone1/android/imu', 'sensor_msgs/Imu');
%............................................................
warning('--Press CTRL-C to end the loop--')

while(true)
    msg = receive(sub, 10); %reecebe os dados da IMU pelo ROS
    quat = [msg.Orientation.W msg.Orientation.X msg.Orientation.Y msg.Orientation.Z]; %quaternio de orientacao dado pela IMU
    rot_matrix = quat2rotm(quat); %transformacao de quaternio para matriz de rotacao (*L1*)
    %Localizacao dos pontos de contato com respeito ao Inercial(*L2*):---------------------------------------------
    p_I_1 = rot_matrix*p_R_1; 
    p_I_2 = rot_matrix*p_R_2;
    p_I_3 = rot_matrix*p_R_3;
    p_I_4 = rot_matrix*p_R_4;
    p_I_5 = rot_matrix*p_R_5;
    p_I_6 = rot_matrix*p_R_6;
    p_I = [p_I_1 p_I_2 p_I_3 p_I_4 p_I_5 p_I_6]; %Os pontos sao guardados numa matriz 'p_I'
    
    for k = 1:(m-1)
        a_t(k,:) = p_I(:,k+1) - p_I(:,k);%Calculo dos eixos de tombamento (arestas do poligono de sustentacao)(*L5*)
    end
    a_t(m,:) = p_I(:,1)-p_I(:,m);
    
    %Normalizando a matriz at---------
    for k = 1:(m)
        a_t(k,:) = a_t(k,:)/norm(a_t(k,:));%Normalizacao de a_t(*L6*)
    end
    %---------------------------------
    for k = 1:(m-1) 
        l(k, :) = (I - a_t(k, :).*transpose(a_t(k, :)))*p_I(:,k+1); %Calculo de l(i) normal a a_t(i) (*L7*)
    end
    l(m, :) = (I - a_t(m, :).*transpose(a_t(m, :)))*p_I(:,1);
    %----------------------------------
    for k = 1:m
        calc =(cross(l(k, :)/norm(l(k, :)),fg/norm(fg)))*transpose(a_t(k, :));%(*L8* e *L9*)
        if calc < 0
            sigma = 1;%(*L10*)
        else
            sigma = -1;%(*L12*)
        end
        Y(k) =sigma*acosd(dot(fg/norm(fg),l(k, :)/norm(l(k, :))));%(*L13*)
    end
    [alpha, alpha_id] = min(Y);%'alpha' recebe o menor dos angulos de capotamento e 'alpha_id' recebe o indice desse angulo
    
    
%----------------------------PLOT--------------------------------------------
    for i = 1: m %plota os pontos de contato 
        plot3(p_I(1,i), p_I(2,i), p_I(3,i), '.','MarkerSize',30, 'Color', 'b');
        hold on
    end
    plot3(o(1), o(2), o(3), '.','MarkerSize', 30, 'Color', 'k'); %plota a origem
    
    %Preenchimento do poligono de sustentacao:------------------------------
    Xfill = [p_I_1(1) p_I_2(1) p_I_3(1) p_I_4(1) p_I_5(1) p_I_6(1)];
    Yfill = [p_I_1(2) p_I_2(2) p_I_3(2) p_I_4(2) p_I_5(2) p_I_6(2)];
    Zfill = [p_I_1(3) p_I_2(3) p_I_3(3) p_I_4(3) p_I_5(3) p_I_6(3)];
    fill3(Xfill, Yfill, Zfill, [0.68 0.92 1],'Facealpha', 0.5);
    daspect([1 1 1])
    
    %Label nos pontos___________
    %!!!OBS: alguns parametros de posicoes dos labels foram definidos apenas analisando visualmente o resultado!!!
    dc = 0.035; %espaco entre o ponto e o respectivo label 
    xt = [p_I_1(1)+dc];
    yt = [p_I_1(2)+dc];
    zt = [p_I_1(3)+dc];
    str = '$${p}_{C(1)}^I$$';
    text(xt,yt,zt,str, 'Interpreter','Latex', 'Color', 'b', 'fontsize',13);
    %
    xt = [p_I_2(1)+dc];
    yt = [p_I_2(2)+dc];
    zt = [p_I_2(3)+dc];
    str = '$${p}_{C(2)}^I$$';
    text(xt,yt,zt,str, 'Interpreter','Latex', 'Color', 'b', 'fontsize',13);
    %
    xt = [p_I_3(1)-2*dc];
    yt = [p_I_3(2)-2*dc];
    zt = [p_I_3(3)+dc];
    str = '$${p}_{C(3)}^I$$';
    text(xt,yt,zt,str, 'Interpreter','Latex', 'Color', 'b', 'fontsize',13);
    %
    xt = [p_I_4(1)-3*dc];
    yt = [p_I_4(2)+dc];
    zt = [p_I_4(3)+dc];
    str = '$${p}_{C(4)}^I$$';
    text(xt,yt,zt,str, 'Interpreter','Latex', 'Color', 'b', 'fontsize',13);
    %
    xt = [p_I_5(1)+dc];
    yt = [p_I_5(2)+dc];
    zt = [p_I_5(3)+dc];
    str = '$${p}_{C(5)}^I$$';
    text(xt,yt,zt,str, 'Interpreter','Latex', 'Color', 'b', 'fontsize',13);
    %
    xt = [p_I_6(1)+dc];
    yt = [p_I_6(2)+dc];
    zt = [p_I_6(3)+dc];
    str = '$${p}_{C(6)}^I$$';
    text(xt,yt,zt,str, 'Interpreter','Latex','Color', 'b', 'fontsize',13);
    %
    xt = [o(1)];
    yt = [o(2)];
    zt = [-0.09];
    str = '$$\hat{f_g}$$';
    text(-xt,yt,zt,str,'Interpreter','Latex', 'Color', 'r', 'fontsize',13); 
    %
    plot3([p_I_1(1) p_I_2(1) p_I_3(1)],[p_I_1(2) p_I_2(2) p_I_3(2)], [p_I_1(3) p_I_2(3) p_I_3(3)], 'Color', 'k');
    plot3([p_I_3(1) p_I_4(1) p_I_5(1)],[p_I_3(2) p_I_4(2) p_I_5(2)], [p_I_3(3) p_I_4(3) p_I_5(3)],'Color', 'k');
    plot3([p_I_5(1) p_I_6(1) p_I_1(1)],[p_I_5(2) p_I_6(2) p_I_1(2)], [p_I_5(3) p_I_6(3) p_I_1(3)],'Color', 'k');
    
    %Plot vetor fg como arrow:------------------------------------------------------------------------------------
    quiver3(o(1), o(2), o(3), fg(1),fg(2), fg(3)/10, 'Color', 'r', 'LineWidth',1.5); %O vetor fg foi reduzido no plot para melhorar a visualizacao
    %-------------------------------------------------------------------------------------------
    
    %Plot dos tracejados ligando os pontos a origem:
    plot3([o(1) p_I_1(1)], [o(2) p_I_1(2)], [o(3) p_I_1(3)], '--', 'Color', 'b');
    plot3([o(1) p_I_2(1)], [o(2) p_I_2(2)], [o(3) p_I_2(3)], '--', 'Color', 'b');
    plot3([o(1) p_I_3(1)], [o(2) p_I_3(2)], [o(3) p_I_3(3)], '--', 'Color', 'b');
    plot3([o(1) p_I_4(1)], [o(2) p_I_4(2)], [o(3) p_I_4(3)], '--', 'Color', 'b');
    plot3([o(1) p_I_5(1)], [o(2) p_I_5(2)], [o(3) p_I_5(3)], '--', 'Color', 'b');
    plot3([o(1) p_I_6(1)], [o(2) p_I_6(2)], [o(3) p_I_6(3)], '--', 'Color', 'b');
    %______________________
    %Plot do vetor l(alpha_id):
    quiver3(o(1), o(2), o(3), l(alpha_id, 1), l(alpha_id, 2), l(alpha_id, 3), 'Color', 'g', 'LineWidth',1.5);
    
    %Label para o vetor l(alpha_id):
    xt = [l(alpha_id,1)-0.016];
    yt = [l(alpha_id,2)];
    zt = [l(alpha_id,3)+0.016];
    str1 = '$$l_';
    str2 = sprintf('%d$$',alpha_id);
    str = strcat(str1,str2);
    text(xt,yt,zt,str,'Interpreter','Latex', 'Color', 'g', 'fontsize',15);
    
    %Plot de arco para indicar o Angulo:---------------------------------------------------------------------------------
    CreateCurvedArrow3([o(1) o(2) (-0.05/2)], [l(alpha_id,1) l(alpha_id,2) l(alpha_id,3)], [0 0 0]); %from, to, center
    xt = [l(alpha_id,1)/8+0.01];
    yt = [l(alpha_id,2)/32];
    zt = [-0.05/2-0.01];
    str1 = '$$\gamma_';
    str2 = sprintf('%d$$',alpha_id);
    str = strcat(str1,str2);
    text(-xt,yt,zt,str,'Interpreter','Latex', 'Color', 'm', 'fontsize',13);
    
    %Parametros de visualizacao do plot:----------------------------------------------------------------------------
    axis([-0.3 0.3 -0.3 0.3 -0.3 0.3]);%limitacao das bordas do grafico
    grid on;
    xlabel('x[m]');
    ylabel('y[m]');
    zlabel('z[m]');
    %view(0, 0)%ver apenas eixos x e z
    %view(90,0); axis([-0.3 0.3 -0.3 0.3 -0.4 0.3]);%ver apenas eixos y e z, alem de imprimir os ângulos de capotamento na parte inferior do gráfico
    hold off;
    %-----------------------------------------------------------
end


%Funcao retirada da internet para plotar arco que representa angulos.
%Fonte: < https://stackoverflow.com/questions/25895072/curved-arrows-in-matlab >
function [h] = CreateCurvedArrow3(from, to, center, count)
%[        
    % Inputs
    if (nargin < 4), count = 15; end
    if (nargin < 3), center = [0 0 0]; end
    center = center(:); from = from(:); to = to(:);

    % Start, stop and normal vectors    
    start = from - center; rstart = norm(start);
    stop = to - center; rstop = norm(stop);
    angle = atan2(norm(cross(start,stop)), dot(start,stop));
    normal = cross(start, stop); normal = normal / norm(normal);

    % Compute intermediate points by rotating 'start' vector
    % toward 'end' vector around 'normal' axis
    % See: http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation/
    phiAngles = linspace(0, angle, count);
    r = linspace(rstart, rstop, count) / rstart;
    intermediates = zeros(3, count);
    a = center(1); b = center(2); c = center(3);
    u = normal(1); v = normal(2); w = normal(3); 
    x = from(1); y = from(2); z = from(3);
    for ki = 1:count
        phi = phiAngles(ki);
        cosp = cos(phi); sinp = sin(phi);
        T = [(u^2+(v^2+w^2)*cosp)  (u*v*(1-cosp)-w*sinp)  (u*w*(1-cosp)+v*sinp) ((a*(v^2+w^2)-u*(b*v+c*w))*(1-cosp)+(b*w-c*v)*sinp); ...
             (u*v*(1-cosp)+w*sinp) (v^2+(u^2+w^2)*cosp)   (v*w*(1-cosp)-u*sinp) ((b*(u^2+w^2)-v*(a*u+c*w))*(1-cosp)+(c*u-a*w)*sinp); ...   
             (u*w*(1-cosp)-v*sinp) (v*w*(1-cosp)+u*sinp)  (w^2+(u^2+v^2)*cosp)  ((c*(u^2+v^2)-w*(a*u+b*v))*(1-cosp)+(a*v-b*u)*sinp); ...
                      0                    0                      0                                1                               ];
        intermediate = T * [x;y;z;r(ki)];
        intermediates(:,ki) = intermediate(1:3);
    end

    % Draw the curved line
    % Can be improved of course with hggroup etc...
    X = intermediates(1,:);
    Y = intermediates(2,:);
    Z = intermediates(3,:);    
    tf = ishold;
    if (~tf), hold on; end
    h = line(X,Y,Z,'Color', 'm', 'LineWidth',1.5);       
    quiver3(X(end-1), Y(end-1), Z(end-1), X(end)-X(end-1), Y(end)-Y(end-1), Z(end)-Z(end-1),1, 'Color', 'm', 'LineWidth',2.5);    
    if (~tf), hold off; end
%]
end


