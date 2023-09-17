clear; close all; clc;

%Para alterar o sistema basta alterar os valores de nD e nE, que são os
%numeros de voltas na roda direita (nD) e na roda esquerda(nE). Valores
%diferentes levam o robô a fazer uma curva para um dos lados e o valor da
%velocidade angular do robo vai ser positiva ou negativa dependendo na
%direção que estiver girando. Se os valores de nD e nE forem iguais, a
%velocidade angular é nula, pois o robô está seguindo em frente.

%dados temporais e de rotação do robô

nD = 5;
nE = 6;

t = 0:25; %pontos para geração de dados
tf = length(t);  %tempo final
numero_de_rotD = nD * 360 * pi/180; % numero de rotações em radianos
numero_de_rotE = nE * 360* pi/180;

%dados do chaci do robô
diametro = 10.5;
l = diametro/2;
r = 3.3;

%angulo da referencia inicial
phi = 0;
phi_rad = phi * pi/180;
xi = 0;
yi = 0;

%jacobiano
J = [r*cos(phi_rad)/2 r*cos(phi_rad)/2;
     r*sin(phi_rad)/2 r*sin(phi_rad)/2;
     r/(2*l) -r/(2*l)];

%equações de angulo de rotação da roda direita do robô
tetaD  = ((3 * numero_de_rotD .* t.^2)/tf^2) - ((2*numero_de_rotD.*t.^3)/tf^3);
dtetaD = ((6 * numero_de_rotD .* t)/tf^2) - ((6*numero_de_rotD.*t.^2)/tf^3);
ddtetaD= (6 * numero_de_rotD/tf^2) - (12*numero_de_rotD.*t/tf^3);

%equações de angulo de rotação da roda esquerda do robô
tetaE  = ((3 * numero_de_rotE .* t.^2)/tf.^2) - ((2*numero_de_rotE.*t.^3)/tf^3);
dtetaE = ((6 * numero_de_rotE .* t)/tf.^2) - ((6*numero_de_rotE.*t.^2)/tf^3);
ddtetaE= (6 * numero_de_rotE/tf.^2) - (12*numero_de_rotE.*t/tf^3);

%iterações
dx  = [ ];
dy  = [ ];
dphi= [ ];

%primeira ieração
dq = [dtetaD(1) dtetaE(1)]';
dp = J*dq;
dx(1)  = dp(1);
dy(1)  = dp(2);
dphi(1)= dp(3);

vetor_phi(1) = phi_rad;
vetor_x(1)   = xi;
vetor_y(1)   = yi;

%segunda iteração com o reajuste do jacobiano
for i = 2:tf

    phi_rad = phi_rad + dp(3);
    xi      = xi + dp(1);
    yi      = yi + dp(2);

    vetor_phi(i) = phi_rad;
    vetor_x(i)   = xi;
    vetor_y(i)   = yi;
    
    J = [r*cos(phi_rad)/2 r*cos(phi_rad)/2;
         r*sin(phi_rad)/2 r*sin(phi_rad)/2;
                  r/(2*l) -r/(2*l)       ];
    
    dq = [dtetaD(i) dtetaE(i)]';
    dp = J*dq;

    dx(i) = dp(1);
    dy(i) = dp(2);
    dphi(i)=dp(3);
    
end

figure(1);
plot(t,dx);
xlabel("t");
ylabel("Vx");

figure(2);
plot(t,dy);
xlabel("t");
ylabel("Vy");

figure(3);
plot(t,dphi);
xlabel("t");
ylabel("Dphi");

figure(4);
plot(t,vetor_phi);
xlabel("t");
ylabel("phi");

figure(5);
plot(vetor_x,vetor_y);
xlabel("X");
ylabel("Y");

figure(6);
plot(dx,dy);
xlabel("dX");
ylabel("dY");

figure(7);
quiver(vetor_x,vetor_y,dx,dy,2,'filled');
xlabel("dX");
ylabel("dY");
