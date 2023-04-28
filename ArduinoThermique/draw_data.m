clear all;close all;
load_data;
t0=0;t1=inf; % temps de visualisation 
clc;
Tech=0.05;fEch=1/Tech;
% trace de U et pos
u=data(:,2); 
TdegC=data(:,1);
N=length(u);
t=Tech*(1:N);
t=transpose(t);
k=find((t>=t0)&(t<=t1)&(TdegC<100));
u=u(k);TdegC=TdegC(k);t=t(k);
figure;
subplot(2,1,1);plot(t,u); grid on;title('u en volts');
subplot(2,1,2);plot(t,TdegC); grid on;title('temperature  en degres celsius');

