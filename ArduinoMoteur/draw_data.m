clear all;close all;
load_data;
clc;
Tech=0.01;fEch=1/Tech;
% trace de U et pos
u=data(:,2); 
pos=data(:,1);
%pos=pos*2*pi/(32*120)% pos en rad sachant qu'on a 32 pas /tour et un reducteur par 120
N=length(u);
t=Tech*(1:N);
t=transpose(t);
figure;
subplot(2,1,1);plot(t,u); grid on;title("command");
subplot(2,1,2);plot(t,pos); grid on;

Te = 0.01;
Fe = 1/Te;

tau = 2;
wc = 1/tau;
wc_t = tan(wc*Te/2)*(2/Te);
tau_t = 1/wc_t;

Fp = tf([1 0],[1 wc_t])
Fz = c2d(Fp,Te,'tustin')

figure;
bode(Fz)

p = tf([1 0],[1]);

we = 2*pi*Fe;
wu = 70;
wi = wu/10;
wi_t = tan(wi*Te/2)*(2/Te);
Cpi_t = (p+wi_t)/(p)
Cpi_z = c2d(Cpi_t,Te,'tustin')

figure;
bode(Cpi_z)

