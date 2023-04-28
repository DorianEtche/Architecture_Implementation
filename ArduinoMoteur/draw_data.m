clear all;close all;
load_data;
clc;
Tech=0.01;fEch=1/Tech;
% trace de U et pos
u=data(:,2); 
pos=data(:,1);
pos=pos*2*pi/(32*120)% pos en rad sachant qu'on a 32 pas /tour et un reducteur par 120
N=length(u);
t=Tech*(1:N);
t=transpose(t);
figure;
subplot(2,1,1);plot(t,u); grid on;
subplot(2,1,2);plot(t,pos); grid on;

