% Metodo de modelagem baseado no Artigo do NXTWay-GS%
clear;clc;
g=9.81;
m=0.023;
R=0.06;
Jw=m*R^2/2;
M=0.636;
W=0.15;
D=0.06;
H=0.22;
L=H/2;
Jv=M*L^2/3;
Jo=M*(W^2+D^2)/12;
Jm=1*10^-5;
Rm=6.69;
Kb=0.468;
Kt=0.317;
n=1;
fm=0.0022;
fw=0;

alpha=n*Kt/Rm;
beta=n*Kt*Kb/Rm + fm;

E=[(2*m+M)*R^2+2*Jw+2*n^2*Jm   M*L*R-2*n^2*Jm
    M*L*R-2*n^2*Jm             M*L^2+Jv+2*n^2*Jm];

A1=[0 0 1 0;0 0 0 1];
A1(3,2)=-g*M*L*E(1,2)/det(E);
A1(4,2)=g*M*L*E(1,1)/det(E);
A1(3,3)=-2*[(beta+fw)*E(2,2)+beta*E(1,2)]/det(E);
A1(4,3)=2*[(beta+fw)*E(1,2)+beta*E(1,1)]/det(E);
A1(3,4)=2*beta*[E(2,2)+E(1,2)]/det(E);
A1(4,4)=-2*beta*[E(1,1)+E(1,2)]/det(E);
B1(3)=alpha*[E(2,2)+E(1,2)]/det(E);
B1(4)=-alpha*[E(1,1)+E(1,2)]/det(E);
B1=B1';
B1(:,2)=B1;

C1=eye(4);
D1=[0 0 0 0;0 0 0 0]';

%Inicia parâmetros da simulação%
AngRoda=0;
AngCorpo=0;
VelAngRoda=0;
VelAngCorpo=0;
vel1=0;
vel2=0;
AngRot=0;
VelAngRot=0;

x1=[AngRoda AngCorpo VelAngRoda VelAngCorpo]';
x2=[AngRot VelAngRot]';
u=[vel1 vel2]';

%Gera sinal de degrau nos estados x;
x1=zeros(4,1000);
x1(2,10)=1*pi/180; % Gera um degrau de 1 grau no angulo do corpo do Segway%
%Simula em malha aberta%
dt=0.01;
for i=1:100
    X(:,i)=A1*x1(:,i)+B1*u;
    if i>9
        x1(:,i+1)=X(:,i);
    end
end
subplot(2,2,1)
plot(X(1,:))
ylabel('velocidade angular do corpo (°/s)');
xlabel('time (s)');
subplot(2,2,2)
plot(X(2,:))
ylabel('posição angular do corpo (°)');
xlabel('time (s)');
subplot(2,2,3)
plot(X(3,:))
ylabel('velocidade angular da roda(°/s)');
xlabel('time(s)');
subplot(2,2,4)
plot(X(4,:))
ylabel('posição angular da roda (°)');
xlabel('time (s)');


%Projeta o controlador
Q = [0.6    0     0     0
     0    35000   0     0 
     0      0   0.001   0
     0      0     0    40];


R = eye(2,2)*10000;

[Klqr,S,e]=lqr(A1,B1,Q,R);

%Gera sinal de degrau nos estados x;
x1=zeros(4,1000);
x1(2,10)=1*pi/180; % Gera um degrau de 1 grau no angulo do corpo do robô
%Simula em malha fechada
dt=0.01;
for i=1:100
    X(:,i)=A1*x1(:,i)+B1*u;
    if i>9
        x1(:,i+1)=x1(:,i)+X(:,i)*dt; % Atualiza Estado do robô
    end
    u=-Klqr*[x1(:,i)+X(:,i)*dt]; % Multiplica o ganho pelo estado atual do robô.
    tensao(i,:)=u;
end
figure();
subplot(2,2,1)
plot(X(1,:))
ylabel('velocidade angular do corpo (°/s)');
xlabel('time (s)');
subplot(2,2,2)
plot(X(2,:))
ylabel('posição angular do corpo (°)');
xlabel('time (s)');
subplot(2,2,3)
plot(X(3,:))
ylabel('velocidade angular da roda(°/s)');
xlabel('time (s)');
subplot(2,2,4)
plot(X(4,:))
ylabel('posição angular da roda (°)');
xlabel('time (s)');






