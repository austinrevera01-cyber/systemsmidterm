clear;
clc;
Kb=0.025879;
R=0.611;
Ki=0.0259;
Torquestall=1.02;
loadless=922.581;
J=0.00000335;
b=4.63*10^-6;
tau=0.00305;
L=0.000119;
Jb=0.0000008;
N=299/14;
Vs=12;
V=24;
Jtot=J+Jb*(N*N);
num = [Ki];
den = [J*L b*L+J*R b*R+Ki*Kb];
sys1=Vs*tf(num, den);
den2=[0 J*R b*R+Ki*Kb];
sys2 =Vs*tf(num, den2);
den3 = [Jtot*L b*L+Jtot*R b*R+Ki*Kb];
sys3 = (Vs*tf(num,den3))/N;
figure(1)
hold on
%step(sys1, 'r');
%step(sys2, '--');
step(sys3, '-');
title('motor model')
ylabel('angular velocity (rad/s)')
xlabel('time (sec)')
%legend('with inductance','no inductance')
hold off