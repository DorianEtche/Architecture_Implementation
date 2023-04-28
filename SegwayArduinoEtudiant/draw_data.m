clear all;close all;load_data;
clc;
Tech=0.01;fEch=1/Tech;
t0=0;t1=inf;
t0=5;t1=15;
% tg_thm * 180.0 / M_PI, thm_lr * 180.0 / M_PI, tg_thb * 180.0 / M_PI, 
% gyroX * 180 / M_PI, tg_vthb * 180.0 / M_PI, 
% 100 * typeNotchFilter + 10 * typeTgControl + 1 * typeLRControl
[m,n]=size(data);
t=(0:(m-1))*Tech;
k=find((t>=t0&(t<=t1)));
u_tg=data(:,1);
u_lr=data(:,2);
thm_tg=data(:,3);
thm_lr=data(:,4);
thbdg=data(:,5);
vthbdgs=data(:,6);
vthbflt=data(:,7);
tmp=data(:,8);
typeTgControl=floor(tmp/100);tmp=tmp-100*typeTgControl;
typeLRControl=floor(tmp/10);tmp=tmp-10*typeLRControl;
typeNotch=tmp;

figure();
subplot(3,1,1);plot(t(k),u_tg(k));grid on; hold on;title("u tg(t)Volt ");
subplot(3,1,2);plot(t(k),vthbdgs(k));grid on; hold on;title("v thb(t) dg/s(b), vthbflt(r) ");
plot(t(k),vthbflt(k),'r');
subplot(3,1,3);
plot(t(k),typeNotch(k)+9,'+r');grid on; hold on;
plot(t(k),typeTgControl(k),'ob');grid on; hold on;
plot(t(k),typeLRControl(k)-11,'vk');grid on; hold on;
title("notch filter+9: red, tang control :blue,LeftRightControl-11:black");
% donnees lr moteurs

figure();
subplot(3,1,1);plot(t(k),u_lr(k));grid on; hold on;title("u lr(t)Volt ");
subplot(3,1,2);plot(t(k),thm_lr(k));grid on; hold on;title("thm lr(t) dg ");
subplot(3,1,3);
plot(t(k),typeNotch(k)+9,'+r');grid on; hold on;
plot(t(k),typeTgControl(k),'ob');grid on; hold on;
plot(t(k),typeLRControl(k)-11,'vk');grid on; hold on;
title("notch filter+9: red, tang control :blue,LeftRightControl-11:black");
% donnees tangentielles moteurs
figure();
subplot(3,1,1);plot(t(k),u_tg(k));grid on; hold on;title("u tg(t)Volt ");
subplot(3,1,2);plot(t(k),thm_tg(k));grid on; hold on;title("thm tg (degres) ");
subplot(3,1,3);
plot(t(k),typeNotch(k)+9,'+r');grid on; hold on;
plot(t(k),typeTgControl(k),'ob');grid on; hold on;
plot(t(k),typeLRControl(k)-11,'vk');grid on; hold on;
title("notch filter+9: red, tang control :blue,LeftRightControl-11:black");

% trace des ffts pour y voir plus clair
N=length(k);
f=fEch*(0:(N-1))/N; 
kf=find(f<=fEch/2);
fftv=fft(vthbdgs(k));
abs_fftv=abs(fftv+1e-8);
fftvflt=fft(vthbflt(k));
abs_fftvflt=abs(fftvflt+1e-8);
figure;
plot(f(kf),abs_fftv(kf));grid on; hold on; title(" fft vthb et vthb filtree (dg/s)"); xlabel("f en Hz");
plot(f(kf),abs_fftvflt(kf),'r');grid on; hold on;

clear data;