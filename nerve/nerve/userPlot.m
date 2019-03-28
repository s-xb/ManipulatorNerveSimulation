clear;
close;
clc;


ADDq=textread('actuDDq.txt','%f');
ADq=textread('actuDq.txt','%f');
Aq=textread('actuq.txt','%f');

q=textread('refq.txt','%f');
Dq=textread('refDq.txt','%f');
DDq=textread('refDDq.txt','%f');

Err=textread('err.txt','%f');
DErr=textread('Derr.txt','%f');

torque=textread('torque.txt','%f');

fi=textread('fi.txt','%f');

plot(Dq,'r');
hold on;
plot(ADq,'g');
plot(DErr,'b');
title('angle velocity');

figure;

plot(q,'r');
hold on;
plot(Aq,'g');
plot(Err,'b');
title('angle position');

figure;
plot(torque,'r');
title('torque');

figure;
plot(fi,'r');
title('fi');