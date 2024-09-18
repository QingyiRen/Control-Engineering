close all
clear
%% Define hydraulic servo actuator transfer function
s =tf ('s');
G=(s^2+5*s+50)/(0.1*s+1)/(s^2+2.5*s+25);
%% Bode gragh of hydraulic servo actuator
 w = logspace(-1,3,200);
 num = [1 5 50];
 f1 = [0.1 1];
 f2 = [1 2.5 25];
 den = conv(f1,f2);
 sys = tf(num,den);
%  bode(sys,w)
 Plant=G;

 figure
 t = 0:0.005:10;
 F = step(Plant,t);
 S_plant = stepinfo(F,t)
 dcgain(Plant)
 clooptf = Plant/(1+Plant);

 allmargin(Plant)
 figure
 margin(Plant)
 %legend('Open loop', 'Closed loop')

 figure
 %step(systf)
 step(minreal(clooptf))
 %legend('Open loop', 'Closed loop')

 figure
 %nyquist(systf)
 nyquist(Plant)
 rlocus(sys)
 %legend('Open loop', 'Closed loop')
%% zeros and poles
zeros=roots(num);
poles=roots(den);
%zeros location
real_z1 = real(zeros);
imag_z1 = imag(zeros);
%poles location
real_z2 = real(poles);
imag_z2 = imag(poles);
figure
plot(real_z1,imag_z1,'bo',real_z2,imag_z2,'r*','MarkerSize',10);
title('Pole-zero map');
xlabel('Real Axis') ;
ylabel('Imaginary Axis') ;
% plot(zeros,'b--o',poles,'c*');
%% Define steady error
s =tf ('s');
G_cl=1/(1+G);
%% Continuous-time Reference Tracking(PI)

%Ts=1.6;
sk=1;
Kp = 0;
Ki = 0.74*sk;
Kd = 0;
t = 0;
CPI1 = pid(Kp,Ki,Kd,t);
% L1=Con1*100/10*(s+10)/(s+100)/(s+4)*0.7/7*(s+7)/(s+0.7)*Plant;%1/3*(s+3)/(s+1)%*2.2/0.5*(s+0.5)/(s+2.2)
L1=CPI1*(s+10)/(s^2+5*s+13.1276)*Plant;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
figure
t = 0:0.005:10;
F = step(Lcl1,t);
S = stepinfo(F,t)
dcgain(Lcl1)
clooptf = L1/(1+L1);
allmargin(L1)
figure
margin(L1)
figure
step(minreal(clooptf))
figure
nyquist(L1)

%Ts=1;
sk=1;
Kp = 0;
Ki = 2.42*sk;
Kd = 0;
t = 0;
CPI2 = pid(Kp,Ki,Kd,t);
% L1=Con1*100/10*(s+10)/(s+100)/(s+4)*0.7/7*(s+7)/(s+0.7)*Plant;%1/3*(s+3)/(s+1)%*2.2/0.5*(s+0.5)/(s+2.2)
L1=CPI2*(s+10)/(s^2+8*s+33.6)*Plant;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
figure
t = 0:0.005:10;
F = step(Lcl1,t);
S = stepinfo(F,t)
dcgain(Lcl1)
clooptf = L1/(1+L1);
allmargin(L1)
figure
margin(L1)
figure
step(minreal(clooptf))
figure
nyquist(L1)


%Ts=1.6;
L1=0.475*(s+10)/s/(s+5)*Plant;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
figure
t = 0:0.005:10;
F = step(Lcl1,t);
S = stepinfo(F,t)
dcgain(Lcl1)
clooptf = L1/(1+L1);
allmargin(L1)
figure
margin(L1)
figure
step(minreal(clooptf))
figure
nyquist(L1)


%Ts=1;
L1=0.79*(s+10)/s/(s+8)*Plant;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
figure
t = 0:0.005:10;
F = step(Lcl1,t);
S = stepinfo(F,t)
dcgain(Lcl1)
clooptf = L1/(1+L1);
allmargin(L1)
figure
margin(L1)
figure
step(minreal(clooptf))
figure
nyquist(L1)

%Ts=1;
C_PID=0.8*(s+10)/s/(s+8.5);
L1=C_PID*Plant;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
figure
t = 0:0.005:10;
F = step(Lcl1,t);
S = stepinfo(F,t)
dcgain(Lcl1)
clooptf = L1/(1+L1);
allmargin(L1)
figure
margin(L1)
figure
step(minreal(clooptf))
figure
nyquist(L1)
%%
% Input Disturbance Rejection
L2=minreal(Plant);
allmargin(L2)
figure
margin(L2)
figure
nyquist(L2)

% I system
Ki = 5000;
Con1=Ki/s;%+Kd*s/(s+wco);%(s+10)/(s*s*(s+5));%1000/s/s+Kd*s/(s+wco);%*(Kp+Ki/s+Kd*s/(s+wco));
Lcl2=L2/(1+L2*Con1);
t = 0:0.005:10;
F = step(Lcl2,t);
S = stepinfo(F,t)
dcgain(Lcl2)
allmargin(Lcl2)
figure
margin(Lcl2)
figure
step(Lcl2)


% PI system
Kp = 100;
Ki = 8000;
% L1=Con1*100/10*(s+10)/(s+100)/(s+4)*0.7/7*(s+7)/(s+0.7)*Plant;%1/3*(s+3)/(s+1)%*2.2/0.5*(s+0.5)/(s+2.2)
% L1=Con1*(s+10)/(s^2+5*s+13.1276)*Plant;
%open loop
Con1=Ki/s+Kp;%+Kd*s/(s+wco);%(s+10)/(s*s*(s+5));%1000/s/s+Kd*s/(s+wco);%*(Kp+Ki/s+Kd*s/(s+wco));
Lcl2=L2/(1+L2*Con1);
figure
t = 0:0.005:10;
F = step(Lcl2,t);
S = stepinfo(F,t)
allmargin(Lcl2)
dcgain(Lcl2)
figure
margin(Lcl2)
figure
step(Lcl2)
figure
nyquist(Lcl2)
% PID system

%Initial guess
Kp = 10;%100;
Ki = 5000;
Kd = 1;%500;
wco= 100;
%open loop
CPIDd=Ki/s+Kp+Kd*s/(s+wco);%+Kd*s/(s+wco);%(s+10)/(s*s*(s+5));%1000/s/s+Kd*s/(s+wco);%*(Kp+Ki/s+Kd*s/(s+wco));
Lcl2=L2/(1+L2*CPIDd);
t = 0:0.005:10;
F = step(Lcl2,t);
S = stepinfo(F,t)
dcgain(Lcl2)
%figure
allmargin(Lcl2)
% figure
% margin(Lcl2)
figure
step(Lcl2)
hold on

%First iteration
Kp = 100;%100;
Ki = 5000;
Kd = 1;%500;
wco= 100;
%open loop
CPIDd=Ki/s+Kp+Kd*s/(s+wco);%+Kd*s/(s+wco);%(s+10)/(s*s*(s+5));%1000/s/s+Kd*s/(s+wco);%*(Kp+Ki/s+Kd*s/(s+wco));
Lcl2=L2/(1+L2*CPIDd);
t = 0:0.005:10;
F = step(Lcl2,t);
S = stepinfo(F,t)
dcgain(Lcl2)
%figure
allmargin(Lcl2)
% figure
% margin(Lcl2)
% figure
hold on
step(Lcl2)

%Final choice
Kp = 100;%100;
Ki = 8000;
Kd = 5;%500;
wco= 100;
%open loop
CPIDd=Ki/s+Kp+Kd*s/(s+wco);%+Kd*s/(s+wco);%(s+10)/(s*s*(s+5));%1000/s/s+Kd*s/(s+wco);%*(Kp+Ki/s+Kd*s/(s+wco));
Lcl2=L2/(1+L2*CPIDd);
t = 0:0.005:10;
F = step(Lcl2,t);
S = stepinfo(F,t)
dcgain(Lcl2)
%figure
allmargin(Lcl2)
% figure
% margin(Lcl2)
% figure
hold on
step(Lcl2)

title('Step responses for disturbance rejection');
legend('Initial controller guess','First iteration controller','Final controller choice')


%%
% Discrete-time control
% controllable canonical realization
[A,B,C,D]=tf2ss(num,den);
%[A,B,C,D]=tf2ss([0 0 0 1],[1 1 0 0]);
%--------------------%
% A=[-12.5 -50 -250;
%       1   0   0  ;
%       0   1   0  ;];
% B=[1 ;0 ;0];
% C=[10 50 500];
% D=0;
%--------------------%
sysc=ss(A,B,C,D);
T_rise=0.595;
h=T_rise/8;
ss_dis=c2d(sysc,h);
Ts1=0.0325;
%roots([1 22 221.5 1570 8024 27610 72160 85000])
%fastest pole -8.4497 , corresponding sampling frequency
%rise time 0.26
Ts2=0.0005;%0.00051;%0.0451;
%roots([1 6125 2.878*10^5 1 1.215*10^7  1.762*10^8 1.526*10^9 9.753*10^9 3.226*10^10  1*10^11])
%fatest pole -6.0776 + 0.0000i, corresponding sampling frequency
%rise time 0.361
% Discrete-time State-Space Model using ZOH and h=0.0181 for set-point
% tracking
sysd_tra=c2d(sysc,Ts1);
phi_tra=sysd_tra.A;
gamma_tra=sysd_tra.B;
% Discrete-time State-Space Model using ZOH and h=0.1 for disturbance
% rejection
sysd_dr=c2d(sysc,Ts2);
phi_dr=sysd_dr.A;
gamma_dr=sysd_dr.B;


%% Discretized controllers and closed-loop response

Cs_PID=c2d(C_PID,Ts1,'Tustin'); %Discrete Controller for set-point step 
Gd1=c2d(Plant,Ts1); %Discrete plant tranfer function
Ld_cl=feedback(Gd1*Cs_PID,1); %Discrete closed-loop system for reference 
%set-point step response 
figure
step(Ld_cl,clooptf); %Compare continuous and discrete-time closed-loop
% response for reference set-point step
title('Step responses for reference tracking');
legend('discrete time','continuous time')
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
% rejection 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); %Discrete closed-loop system for disturbance
% rejection
figure
step(Ld_dist,Lcl2); %Compare continuous and discrete-time closed-loop
% response for disturbance rejection
title('Step responses for disturbance rejection');
legend('discrete time','continuous time')

%--------------the choice of optimal sampling time for Ts2-------------%
Ts2=0.0001;
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
% rejection 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); %Discrete closed-loop system for disturbance
% rejection
figure
step(Ld_dist); %Compare continuous and discrete-time closed-loop
hold on
Ts2=0.0002;
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
% rejection 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); %Discrete closed-loop system for disturbance
% rejection
step(Ld_dist); %Compare continuous and discrete-time closed-loop
Ts2=0.0005;
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
% rejection 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); %Discrete closed-loop system for disturbance
% rejection
step(Ld_dist); %Compare continuous and discrete-time closed-loop
hold on
Ts2=0.0008;
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
% rejection 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); %Discrete closed-loop system for disturbance
% rejection
step(Ld_dist); %Compare continuous and discrete-time closed-loop
Ts2=0.001;
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
% rejection 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); %Discrete closed-loop system for disturbance
% rejection
step(Ld_dist); %Compare continuous and discrete-time closed-loop
step(Lcl2);
title('Step responses for disturbance rejection');
legend('Ts2=0.0001','Ts2=0.0002','Ts2=0.0005','Ts2=0.0008','Ts2=0.001','continuous time')
Ts2=0.0005;


%% State-Feedback servo tracking 
Co=ctrb(sysd_tra); % Check system controllability
rank(Co) %Rank=Length(A) so the system is controllabl feedback servo tracking
% Kalman decomposition is used to check controllable modes
[~,~,~,~,k]=ctrbf(phi_tra,gamma_tra,C); % sum(k)=3 so there are 3 controllable modes
% thus the system is controllable

% pole set 1
Ts1=0.599/16;
% ps1=[0.9998-0.003i 0.9998+0.0003i 0.9996];
% Ks=place(phi_tra,gamma_tra,ps1);
% Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
% phi_cl=phi_tra-gamma_tra*Ks;
% sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
% gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
% sys_poleset1=ss(phi_cl,gamma_cl,C,D,Ts1);
% figure
% step(sys_poleset1) % step response

% pole set 2

ps2=[0.9988-0.0013i 0.9988+0.0013i 0.9633];
Ks=place(phi_tra,gamma_tra,ps2);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset2=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset2) % step response


% pole set 3

ps3=[0.9976-0.0025i 0.9976+0.0025i 0.9633];
Ks=place(phi_tra,gamma_tra,ps3);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset3=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset3) % step response

% pole set 4

ps4=[0.9952-0.005i 0.9952+0.005i 0.9633];
Ks=place(phi_tra,gamma_tra,ps4);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset4=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset4) % step response

% pole set 5

ps5=[0.9880-0.0125i 0.0027+0.0125i 0.9633];
Ks=place(phi_tra,gamma_tra,ps5);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset5=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset5) % step response


% pole set 6

ps6=[0.9759-0.0246i 0.9759+0.0246i 0.8293];
Ks=place(phi_tra,gamma_tra,ps6);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset6=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset6) % step response


% pole set 7

ps7=[0.9518-0.0481i 0.9518+0.0481i 0.8293];
Ks=place(phi_tra,gamma_tra,ps7);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset7=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset7) % step response

% pole set 8

ps8=[0.8795-0.1116i 0.8795+0.1116i 0.6877];
Ks=place(phi_tra,gamma_tra,ps8);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset8=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset8) % step response

% pole set 9

ps9=[0.7611-0.1964i 0.7611+0.1964i 0.4730];
Ks=place(phi_tra,gamma_tra,ps9);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset9=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset9) % step response

% % pole set 10
% 
% ps8=[0.5407-0.2989i 0.5407+0.2989i 0.1538];
% Ks=place(phi_tra,gamma_tra,ps8);
% Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
% phi_cl=phi_tra-gamma_tra*Ks;
% sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
% gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
% sys_poleset8=ss(phi_cl,gamma_cl,C,D,Ts1);
% hold on
% step(sys_poleset8) % step response

title('Step responses of the five different controlled systems');
legend('Poles Set 1','Poles Set 2','Poles Set 3','Poles Set 4','Poles Set 5','Poles Set 6','Poles Set 7','Poles Set 8')

figure
step(sys_poleset6)
hold on
step(sys_poleset7) 
hold on
step(sys_poleset8) 
hold on
step(sys_poleset9) 
title('Step responses of the four different controlled systems');
legend('Poles Set 1','Poles Set 2','Poles Set 3','Poles Set 4')



ps8=[0.8795-0.1116i 0.8795+0.1116i 0.6877];
Ks=place(phi_tra,gamma_tra,ps8);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset8=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset8) % step response
t3 = 0:(Ts1):0.15;      % simulation time = 10 seconds
u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y, Tsim, X] = lsim(sys_poleset8,u,t3,[0 0 0]);  % simulate
figure
plot(Tsim,-Ks*X(:,1:3)'+inv(dcgain(sys_closedfb)));
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of State-Feedback controller')
xlim([0 0.15])
figure
step(sys_poleset8) 
%% Output-Feedback reference tracking 


%------------------------------x0=[0.001,0.001,0.001]---------------------------------%
sysd=c2d(sysc,Ts1,'zoh');
%state-feedbak gain design
ps8=[0.8795-0.1116i 0.8795+0.1116i 0.6877];
feedgain=place(sysd.a,sysd.b,ps8); % Gain K
sys_closed_fb_ob = ss(sysd.A - sysd.B*feedgain, sysd.B, sysd.C, sysd.D, Ts1); % Kr
Gff=inv(dcgain(sys_closed_fb_ob));
pf_ob_d=[0.4730 0.4556 0.4227];
L=place(sysd.a',sysd.c',pf_ob_d)'; % Gain L
At = [sysd.A   -sysd.B*feedgain; L*sysd.C  sysd.A-(L*sysd.C)-(sysd.B*feedgain)]; 
Bt = [sysd.B*Gff; sysd.B*Gff] ;   
Ct = [sysd.C 0 0 0;0 0 0 sysd.C];
sys_closed_ff_ob = ss(At,Bt,Ct,sysd.D,Ts1); %output feedback close-loop system


x0 = [0.001 ,0.001, 0.001]; % give initial condition
t3 = 0:(Ts1):15;      % simulation time = 10 seconds

u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y, Tsim, X] = lsim(sys_closed_ff_ob,u,t3,[x0 0 0 0]);  % simulate
figure
stairs(Tsim,Y(:,1))  % plot the output vs. time
hold on;
stairs(Tsim,Y(:,2))  % plot the observer vs. time
title('step response')
legend('System Output','Observer Output')
xlabel('Time (seconds)');
ylabel('Amplitute');
xlim([0 3])
hold off;
figure
%estimation error of true state x1,x2,x3
stairs(Tsim,X(:,1)-X(:,4))
hold on
stairs(Tsim,X(:,2)-X(:,5))
hold on
stairs(Tsim,X(:,3)-X(:,6))
title('Estimation error')
legend('Error of state x1','Error of state x2','Error of state x3')
xlabel('Time (seconds)');
ylabel('Amplitute');
figure
stairs(Tsim,Y(:,1))  % plot the output vs. time
hold on;
stairs(Tsim,Y(:,2))  % plot the observer vs. time
hold on;
step(sys_poleset8,'g')
hold on;
title('step response')
legend('System Output','Observer Output','full state information')

%Control Input of Output-Feedback reference tracking 
figure
plot(Tsim,-feedgain*X(:,1:3)'+Gff);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of Output-Feedback reference tracking')
xlim([0 15])



%% Output-Feedback disturbance rejection 
sysddr=c2d(sysc,Ts2,'zoh');
poles=[0.9984+0.0017i 0.9984-0.0017i 0.99];%change to Ts2
feedgain=place(sysddr.A,sysddr.B,poles); % Gain K
sys_closed = ss(sysddr.A - sysddr.B*feedgain, sysddr.B, sysddr.C, sysddr.D, Ts2); % Kr
poles_d_ob=[0.9896	0.9891	0.9886];% Observer poles

% Define new state matrices for constant load dist 
Ae = [sysddr.A sysddr.B; 0 0 0 0];
Be = [sysddr.B; 0];
Ce = [sysddr.C 0];
De = 0;
% New Feedback gain 
Ke = [feedgain 0.05];
% Design new observer gain
Le = place(Ae',Ce',[poles_d_ob 0.75])';
% Combine
At = [Ae -Be*Ke;Le*Ce Ae-Be*Ke-Le*Ce]; 
Bt = [Be*inv(dcgain(sys_closed));Be*inv(dcgain(sys_closed))] ;
Ct = [Ce 0 0 0 0;0 0 0 0 Ce];
SysObsLoadD = ss(At,Bt,Ct,sysddr.d,Ts2);
Sys_StateD = ss(Ae-Be*Ke,inv(dcgain(sys_closed))*Be,Ce,sysddr.d,Ts2);
Gff=inv(dcgain(Sys_StateD));
% disturbance rejection rt=0 dr=1
% give initial condition
x0 = [0.001 0.001 0.001 1];
o0=  [0 0 0 0];
t3 = 0:Ts2:2;  % simulation time = 1 seconds
% Unit step signal 
u=1;
u(1:length(t3))=1; 
u(2:length(t3))=0;    
[Y, Tsim, X] = lsim(SysObsLoadD,u,t3,[x0 o0]); 
[Y1, Tsim1, X1] = lsim(Sys_StateD,u,t3,x0); 
figure
% b=[Y(:,1),Y(:,2)];
stairs(Tsim,Y(:,1));
hold on
stairs(Tsim,Y(:,2));
u1=1;
u1(1:length(t3))=1;
xlabel('time [s]')
ylabel ( 'Amplitide')
legend({'System Output','Observer Output'})
title('System Output vs Observer Output');

% reference trakcing  rt=1 dr=1
% give initial condition
x0 = [0.001 0.001 0.001 1];
o0=  [0 0 0 0];
t3 = 0:Ts2:2;  % simulation time = 1 seconds
% Unit step signal  
u=1;
u(1:length(t3))=1; 
u(2:length(t3))=1;    
[Y, Tsim, X] = lsim(SysObsLoadD,u,t3,[x0 o0]); 
[Y1, Tsim1, X1] = lsim(Sys_StateD,u,t3,x0); 
figure
stairs(Tsim,Y(:,1));
hold on
stairs(Tsim1,Y(:,2));
hold on
xlabel('time [s]')
ylabel ( 'Amplitide')
legend({'System Output','Observer Output'})
title('System Output vs Observer Output');

figure
stairs(Tsim,X(:,1)-X(:,4))
hold on
stairs(Tsim,X(:,2)-X(:,5))
hold on
stairs(Tsim,X(:,3)-X(:,6))
title('Estimation error')
legend('Error of state x1','Error of state x2','Error of state x3')
xlabel('Time (seconds)');
ylabel('Amplitute');
xlim([0 3])
%Control Input of Output-Feedback controller 
figure
plot(Tsim,-feedgain*X(:,4:6)'-1+Gff);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of Output-Feedback disturbance rejection ')



%% LQ Controller

% changing Q



%Changing Q_11

% Q=0.01
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,1,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd1,S,e] = dlqr(A,B,Q1,R1,0);
GgainD1 = 1/((C-D*Kd1)*inv(eye(3)-(A-B*Kd1))*B+D);
SysD = ss(A-B*Kd1,GgainD1*B,C,0,plantd.Ts);
% Plot figure
figure
step (SysD)
[y,t1,x1] = step(SysD); 
stepinfo(y,t1)
hold on

% Q=1
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd2,S,e] = dlqr(A,B,Q1,R1,0);
GgainD2 = 1/((C-D*Kd2)*inv(eye(3)-(A-B*Kd2))*B+D);
SysD = ss(A-B*Kd2,GgainD2*B,C,0,plantd.Ts);
step (SysD)
hold on
[y,t2,x2] = step(SysD); 
stepinfo(y,t2)
hold on

% Q=100
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [100,1,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd3,S,e] = dlqr(A,B,Q1,R1,0);
GgainD3 = 1/((C-D*Kd3)*inv(eye(3)-(A-B*Kd3))*B+D);
SysD = ss(A-B*Kd3,GgainD3*B,C,0,plantd.Ts);
% Plot figure
step (SysD)
hold on
[y,t3,x3] = step(SysD); 
stepinfo(y,t3)

%Q=10000
mr = 1;
mq = [10000,1,1];
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd4,S,e] = dlqr(A,B,Q1,R1,0);
GgainD4 = 1/((C-D*Kd4)*inv(eye(3)-(A-B*Kd4))*B+D);
SysD = ss(A-B*Kd4,GgainD4*B,C,0,plantd.Ts);


% Plot four Q steps
step (SysD)
hold on
[y,t4,x4] = step(SysD); 
stepinfo(y,t4)
legend('Q=0.01','Q=1','Q=100','Q=10000')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Influence of Q_{11} element')
%control effort
figure
plot(t1,-Kd1*x1'+GgainD1);
hold on;
plot(t2,-Kd2*x2'+GgainD2);
hold on
plot(t3,-Kd3*x3'+GgainD3);
hold on
plot(t4,-Kd4*x4'+GgainD4);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Input control signal of LQ control')
legend({'Q=0.01','Q=1','Q=100','Q=10000'})


%Changing Q_22

% Q=0.01
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,0.01,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd1,S,e] = dlqr(A,B,Q1,R1,0);
GgainD1 = 1/((C-D*Kd1)*inv(eye(3)-(A-B*Kd1))*B+D);
SysD = ss(A-B*Kd1,GgainD1*B,C,0,plantd.Ts);
% Plot figure
figure
step (SysD)
[y,t1,x1] = step(SysD); 
stepinfo(y,t1)
hold on

% Q=1
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd2,S,e] = dlqr(A,B,Q1,R1,0);
GgainD2 = 1/((C-D*Kd2)*inv(eye(3)-(A-B*Kd2))*B+D);
SysD = ss(A-B*Kd2,GgainD2*B,C,0,plantd.Ts);
step (SysD)
hold on
[y,t2,x2] = step(SysD); 
stepinfo(y,t2)
hold on

% Q=100
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,100,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd3,S,e] = dlqr(A,B,Q1,R1,0);
GgainD3 = 1/((C-D*Kd3)*inv(eye(3)-(A-B*Kd3))*B+D);
SysD = ss(A-B*Kd3,GgainD3*B,C,0,plantd.Ts);
% Plot figure
step (SysD)
hold on
[y,t3,x3] = step(SysD); 
stepinfo(y,t3)

%Q=10000
mr = 1;
mq = [1,10000,1];
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd4,S,e] = dlqr(A,B,Q1,R1,0);
GgainD4 = 1/((C-D*Kd4)*inv(eye(3)-(A-B*Kd4))*B+D);
SysD = ss(A-B*Kd4,GgainD4*B,C,0,plantd.Ts);


% Plot four Q steps
step (SysD)
hold on
[y,t4,x4] = step(SysD); 
stepinfo(y,t4)
legend('Q=0.01','Q=1','Q=100','Q=10000')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Influence of Q_{22} element')
%control effort
figure
plot(t1,-Kd1*x1'+GgainD1);
hold on;
plot(t2,-Kd2*x2'+GgainD2);
hold on
plot(t3,-Kd3*x3'+GgainD3);
hold on
plot(t4,-Kd4*x4'+GgainD4);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Input control signal of LQ control')
legend({'Q=0.01','Q=1','Q=100','Q=10000'})




%Changing Q_33

% Q=0.01
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,0.01];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd1,S,e] = dlqr(A,B,Q1,R1,0);
GgainD1 = 1/((C-D*Kd1)*inv(eye(3)-(A-B*Kd1))*B+D);
SysD = ss(A-B*Kd1,GgainD1*B,C,0,plantd.Ts);
% Plot figure
figure
step (SysD)
[y,t1,x1] = step(SysD); 
stepinfo(y,t1)
hold on

% Q=1
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd2,S,e] = dlqr(A,B,Q1,R1,0);
GgainD2 = 1/((C-D*Kd2)*inv(eye(3)-(A-B*Kd2))*B+D);
SysD = ss(A-B*Kd2,GgainD2*B,C,0,plantd.Ts);
step (SysD)
hold on
[y,t2,x2] = step(SysD); 
stepinfo(y,t2)
hold on

% Q=100
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,100];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd3,S,e] = dlqr(A,B,Q1,R1,0);
GgainD3 = 1/((C-D*Kd3)*inv(eye(3)-(A-B*Kd3))*B+D);
SysD = ss(A-B*Kd3,GgainD3*B,C,0,plantd.Ts);
% Plot figure
step (SysD)
hold on
[y,t3,x3] = step(SysD); 
stepinfo(y,t3)

%Q=10000
mr = 1;
mq = [1,1,10000];
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd4,S,e] = dlqr(A,B,Q1,R1,0);
GgainD4 = 1/((C-D*Kd4)*inv(eye(3)-(A-B*Kd4))*B+D);
SysD = ss(A-B*Kd4,GgainD4*B,C,0,plantd.Ts);


% Plot four Q steps
step (SysD)
hold on
[y,t4,x4] = step(SysD); 
stepinfo(y,t4)
legend('Q=0.01','Q=1','Q=100','Q=10000')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Influence of Q_{33} element')
%control effort
figure
plot(t1,-Kd1*x1'+GgainD1);
hold on;
plot(t2,-Kd2*x2'+GgainD2);
hold on
plot(t3,-Kd3*x3'+GgainD3);
hold on
plot(t4,-Kd4*x4'+GgainD4);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Input control signal of LQ control')
legend({'Q=0.01','Q=1','Q=100','Q=10000'})



%Changing Q_2

% Q=0.01
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,1];
mr = 0.01;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd1,S,e] = dlqr(A,B,Q1,R1,0);
GgainD1 = 1/((C-D*Kd1)*inv(eye(3)-(A-B*Kd1))*B+D);
SysD = ss(A-B*Kd1,GgainD1*B,C,0,plantd.Ts);
% Plot figure
figure
step (SysD)
[y,t1,x1] = step(SysD); 
stepinfo(y,t1)
hold on

% Q=1
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,1];
mr = 1;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t,x] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd2,S,e] = dlqr(A,B,Q1,R1,0);
GgainD2 = 1/((C-D*Kd2)*inv(eye(3)-(A-B*Kd2))*B+D);
SysD = ss(A-B*Kd2,GgainD2*B,C,0,plantd.Ts);
step (SysD)
hold on
[y,t2,x2] = step(SysD); 
stepinfo(y,t2)
hold on

% Q=100
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [1,1,1];
mr = 100;
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd3,S,e] = dlqr(A,B,Q1,R1,0);
GgainD3 = 1/((C-D*Kd3)*inv(eye(3)-(A-B*Kd3))*B+D);
SysD = ss(A-B*Kd3,GgainD3*B,C,0,plantd.Ts);
% Plot figure
step (SysD)
hold on
[y,t3,x3] = step(SysD); 
stepinfo(y,t3)

%Q=10000
mr = 10000;
mq = [1,1,1];
Q = diag(mq); 
R = 1*mr;
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 16;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv
A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;
Q1 = Q;
R1 = R;
% dlqr
[Kd4,S,e] = dlqr(A,B,Q1,R1,0);
GgainD4 = 1/((C-D*Kd4)*inv(eye(3)-(A-B*Kd4))*B+D);
SysD = ss(A-B*Kd4,GgainD4*B,C,0,plantd.Ts);


% Plot four Q steps
step (SysD)
hold on
[y,t4,x4] = step(SysD); 
stepinfo(y,t4)
legend('Q=0.01','Q=1','Q=100','Q=10000')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Influence of Q_{2} element')
%control effort
figure
plot(t1,-Kd1*x1'+GgainD1);
hold on;
plot(t2,-Kd2*x2'+GgainD2);
hold on
plot(t3,-Kd3*x3'+GgainD3);
hold on
plot(t4,-Kd4*x4'+GgainD4);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Input control signal of LQ control')
legend({'Q=0.01','Q=1','Q=100','Q=10000'})


%% Question 7: Final

% choosing optimal values for Q_2

A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,10000,0.01];
mr = 0.01;
Q = diag(mq); % Q=diag[1,1,100]
R = mr;% R=0.01
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;

Q1 = Q;
R1 = R;
% dlqr
[Kd,S,e] = dlqr(A,B,Q1,R1,0);
GgainD = 1/((C-D*Kd)*inv(eye(3)-(A-B*Kd))*B+D);
SysD = ss(A-B*Kd,GgainD*B,C,0,h/2);

% Plot figure
figure
step (SysD)
hold on
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,10000,0.01];
mr = 1;
Q = diag(mq); % Q=diag[1,1,100]
R = mr;% R=0.01
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;

Q1 = Q;
R1 = R;
% dlqr
[Kd,S,e] = dlqr(A,B,Q1,R1,0);
GgainD = 1/((C-D*Kd)*inv(eye(3)-(A-B*Kd))*B+D);
SysD = ss(A-B*Kd,GgainD*B,C,0,h/2);

% Plot figure
step (SysD)
legend('Q_2=0.01','Q_2=1')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 5])
title('Choosing optimal value for Q_{2} element')




% Final LQ controller 

A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,10000,0.01];
mr = 1;
Q = diag(mq); % Q=diag[1,1,100]
R = mr;% R=0.01
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;

Q1 = Q;
R1 = R;
% dlqr
[Kd,S,e] = dlqr(A,B,Q1,R1,0);
GgainD = 1/((C-D*Kd)*inv(eye(3)-(A-B*Kd))*B+D);
SysD = ss(A-B*Kd,GgainD*B,C,0,h/2);

% Plot figure
figure
step (SysD)
[y1,t1,x1] = step(SysD); 
stepinfo(y1,t1)
figure
%control effort
plot(t1,-Kd*x1'+GgainD);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of LQ controller')
xlim([0 0.9])


%% Redesign of Discrete-Time PID-Type Controllers

%redesign of reference tracking
C_PID2=0.85*(s+14)/s/(s+13.5);
L1=C_PID2*Plant;
L1=minreal(L1);
figure
margin(L1)
Lcl1=feedback(L1,1);
figure
t = 0:0.005:10;
F = step(Lcl1,t);
S = stepinfo(F,t)
dcgain(Lcl1)
clooptf = L1/(1+L1);
allmargin(L1)
figure
margin(L1)
figure
step(minreal(clooptf))
Ts1_r=0.0403;
Cs_PID=c2d(C_PID,Ts1,'Tustin'); %Discrete Controller for set-point step 
Cs_PID2=c2d(C_PID2,Ts1_r,'Tustin');
Gd1=c2d(Plant,Ts1); %Discrete plant tranfer function
Gd2=c2d(Plant,Ts1_r); %Discrete plant tranfer function
Ld_cl=feedback(Gd1*Cs_PID,1); %Discrete closed-loop system for reference 
Ld_cl2=feedback(Gd2*Cs_PID2,1); %Discrete closed-loop system for reference

figure
step(Ld_cl,Ld_cl2); %Compare continuous and discrete-time closed-loop
% response for reference set-point step
legend('previous controller','redesigned controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')

%redesign of dsiturbance rejection
Kp = 150;%100;
Ki = 12000;
Kd = 20;%500;
wco= 100;
%open loop
CPIDd2=Ki/s+Kp+Kd*s/(s+wco);
Cd_PID=c2d(CPIDd,Ts2,'Tustin'); %Discrete Controller for disturbance 
Cd_PID2=c2d(CPIDd2,Ts2,'Tustin'); 
Gd2=c2d(Plant,Ts2); %Discrete plant tranfer function
Ld_dist=feedback(Gd2,Cd_PID); 
Ld_dist2=feedback(Gd2,Cd_PID2); 
figure
step(Ld_dist,Ld_dist2); %Compare continuous and discrete-time closed-loop
legend('previous controller','redesigned controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')

%% Redesign of Discrete-Time State Feedback Pole Placement Controller

ps8=[0.8795-0.1116i 0.8795+0.1116i 0.6877];
Ks=place(phi_tra,gamma_tra,ps8);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset8=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_poleset8) % step response
t3 = 0:(Ts1):0.15;      % simulation time = 10 seconds
u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y, Tsim, X] = lsim(sys_poleset8,u,t3,[0 0 0]);  % simulate
figure
plot(Tsim,-Ks*X(:,1:3)'+inv(dcgain(sys_closedfb)));
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of State-Feedback controller')
xlim([0 0.15])
figure
step(sys_poleset8) 
%----------Redesign of Discrete-Time State Feedback Pole Placement Controlle------------%
ps_r=[0.8795-0.1116i 0.8795+0.1116i 0.8];
Ks=place(phi_tra,gamma_tra,ps_r);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_polesetr=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_polesetr) % step response
legend('previous controller','redesigned controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')


t3 = 0:(Ts1):0.15;      % simulation time = 10 seconds
u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y, Tsim, X] = lsim(sys_polesetr,u,t3,[0 0 0]);  % simulate
figure
plot(Tsim,-Ks*X(:,1:3)'+inv(dcgain(sys_closedfb)));
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of State-Feedback controller')
xlim([0 0.15])

%% Redesign of Discrete-Time LQ Control
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,10000,0.01];
mr = 1;
Q = diag(mq); % Q=diag[1,1,100]
R = mr;% R=0.01
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;

Q1 = Q;
R1 = R;
% dlqr
[Kd1,S,e] = dlqr(A,B,Q1,R1,0);
GgainD1 = 1/((C-D*Kd1)*inv(eye(3)-(A-B*Kd1))*B+D);
SysD1 = ss(A-B*Kd1,GgainD1*B,C,0,h/2);

%----------------------Redesign of LQ controller--------------------------%
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,6500,0.01];
mr = 1;
Q = diag(mq); % Q=diag[1,1,100]
R = mr;% R=0.01
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;

Q1 = Q;
R1 = R;
% dlqr
[Kd2,S,e] = dlqr(A,B,Q1,R1,0);
GgainD2 = 1/((C-D*Kd2)*inv(eye(3)-(A-B*Kd2))*B+D);
SysD2 = ss(A-B*Kd2,GgainD2*B,C,0,h/2);

% Plot figure
figure
step (SysD1)
hold on
step (SysD2)
legend('previous controller','redesigned controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 1.1])

[y1,t1,x1] = step(SysD1); 
stepinfo(y1,t1)
[y2,t2,x2] = step(SysD2); 
stepinfo(y2,t2)


figure
%control effort
plot(t2,-Kd2*x2'+GgainD2);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of LQ controller')
xlim([0 5])

%% time delay fot State-Feedback reference tracking 

clear zeros
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
ps8=[0.8795-0.1116i 0.8795+0.1116i 0.6877];
Ks=place(phi_tra,gamma_tra,ps8);
Kr=-(phi_tra-gamma_tra*Ks)/(C*gamma_tra);
phi_cl=phi_tra-gamma_tra*Ks;
sys_closedfb=ss(phi_cl,gamma_tra,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_tra;
sys_poleset8=ss(phi_cl,gamma_cl,C,D,Ts1);
t3 = 0:(Ts1):0.15;      % simulation time = 10 seconds
u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y, Tsim, X] = lsim(sys_poleset8,u,t3,[0 0 0]);  % simulate
figure
plot(Tsim,-Ks*X(:,1:3)'+inv(dcgain(sys_closedfb)));
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of State-Feedback controller')
xlim([0 0.15])
figure
step(sys_poleset8) % step response
hold on
% time delay system
ps_delay=[0.8795-0.1116i 0.8795+0.1116i 0.6877 0.5];
phi_delay=[phi_tra gamma_tra;-Ks -0.5];
gamma_delay=[0;gamma_cl];
C=[C 0];
Ks=place(phi_delay,gamma_delay,ps_delay);
Kr=-(phi_delay-gamma_delay*Ks)/(C*gamma_delay);
phi_cl=phi_delay-gamma_delay*Ks;
sys_closedfb=ss(phi_cl,gamma_delay,C,D,Ts1);
gamma_cl=inv(dcgain(sys_closedfb))*gamma_delay;
sys_delay=ss(phi_cl,gamma_cl,C,D,Ts1);
hold on
step(sys_delay) % step response
legend('previous controller','time delayed controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')







%% time delay for Output-Feedback control

% reference tracking
sysd=c2d(sysc,Ts1,'zoh');
%state-feedbak gain design
ps8=[0.8795-0.1116i 0.8795+0.1116i 0.6877];
feedgain=place(sysd.a,sysd.b,ps8); % Gain K
sys_closed_fb_ob = ss(sysd.A - sysd.B*feedgain, sysd.B, sysd.C, sysd.D, Ts1); % Kr
Gff=inv(dcgain(sys_closed_fb_ob));
pf_ob_d=[0.4730 0.4556 0.4227];
L=place(sysd.a',sysd.c',pf_ob_d)'; % Gain L
At = [sysd.A   -sysd.B*feedgain; L*sysd.C  sysd.A-(L*sysd.C)-(sysd.B*feedgain)]; 
Bt = [sysd.B*Gff; sysd.B*Gff] ;   
Ct = [sysd.C 0 0 0;0 0 0 sysd.C];
sys_closed_ff_ob = ss(At,Bt,Ct,sysd.D,Ts1); %output feedback close-loop system


x0 = [0.001 ,0.001, 0.001]; % give initial condition
t3 = 0:(Ts1):15;      % simulation time = 10 seconds

u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y1, Tsim, X] = lsim(sys_closed_ff_ob,u,t3,[x0 0 0 0]);  % simulate

stairs(Tsim,Y1(:,1))  % plot the output vs. time
hold on;
stairs(Tsim,Y1(:,2))  % plot the observer vs. time
title('step response')
legend('System Output','Observer Output')
xlabel('Time (seconds)');
ylabel('Amplitute');
xlim([0 3])

% %Control Input of Output-Feedback reference tracking 
% figure
% plot(Tsim,-feedgain*X(:,1:3)'+Gff);
% xlabel ( 'Time[s]')
% ylabel ( 'Amplitude')
% title('Control Input of Output-Feedback reference tracking')
% xlim([0 15])
% 

clear zeros
sysd=c2d(sysc,Ts1,'zoh');
%state-feedbak gain design
ps8=[0.8-0.1116i 0.8+0.1116i 0.6877];
feedgain=place(sysd.a,sysd.b,ps8); % Gain K
sys_closed_fb_ob = ss(sysd.A - sysd.B*feedgain, sysd.B, sysd.C, sysd.D, Ts1); % Kr
Gff=inv(dcgain(sys_closed_fb_ob));
% pf_ob_d=[2.2804 2.4577 2.6488].*10^(-4);
pf_ob_d=[0.4730 0.4556 0.4227];
L=place(sysd.a',sysd.c',pf_ob_d)'; % Gain L
At = [sysd.A   zeros(3,1) -sysd.B*feedgain; zeros(1,3) -feedgain 0;L*sysd.C zeros(3,1) sysd.A-(L*sysd.C)-(sysd.B*feedgain)]; 
Bt = [sysd.B*Gff; Gff;sysd.B*Gff] ;   
Ct = [sysd.C 0 0 0 0;0 0 0 0 sysd.C];
ps_delayed=[0.8795-0.1116i 0.8795+0.1116i 0.6877 0.5 pf_ob_d];
feedgain_p=place(At,Bt,ps_delayed);
At = [sysd.A   zeros(3,1) -sysd.B*feedgain; -feedgain_p;L*sysd.C zeros(3,1) sysd.A-(L*sysd.C)-(sysd.B*feedgain)]; 
sys_closed_ff_ob = ss(At,Bt,Ct,sysd.D,Ts1); %output feedback close-loop system


x0 = [0.001 ,0.001, 0.001,0]; % give initial condition
t3 = 0:(Ts1):15;      % simulation time = 10 seconds
u=0;
u(1:length(t3))=1;  % Unit step signal 
u(2:length(t3))=1;  % Unit step signal
[Y2, Tsim, X] = lsim(sys_closed_ff_ob,u,t3,[x0 0 0 0]);  % simulate
figure
stairs(Tsim,Y1(:,2))  % plot the output vs. time
hold on;
stairs(Tsim,Y2(:,2))
xlim([0 2])


title('step response')
legend('Original system','Time delayed system')
xlabel('Time (seconds)');
ylabel('Amplitute');



%disturbance rejection
sysddr=c2d(sysc,Ts2,'zoh');
poles=[0.9984+0.0017i 0.9984-0.0017i 0.99];%change to Ts2
feedgain=place(sysddr.A,sysddr.B,poles); % Gain K
sys_closed = ss(sysddr.A - sysddr.B*feedgain, sysddr.B, sysddr.C, sysddr.D, Ts2); % Kr
poles_d_ob=[0.9896	0.9891	0.9886];% Observer poles
% Define new state matrices for constant load dist 
Ae = [sysddr.A sysddr.B; 0 0 0 0];
Be = [sysddr.B; 0];
Ce = [sysddr.C 0];
De = 0;
% New Feedback gain 
Ke = [feedgain 0.05];
% Design new observer gain
Le = place(Ae',Ce',[poles_d_ob 0.75])';
% Combine
At = [Ae -Be*Ke;Le*Ce Ae-Be*Ke-Le*Ce]; 
Bt = [Be*inv(dcgain(sys_closed));Be*inv(dcgain(sys_closed))] ;
Ct = [Ce 0 0 0 0;0 0 0 0 Ce];
SysObsLoadD = ss(At,Bt,Ct,sysddr.d,Ts2);
Sys_StateD = ss(Ae-Be*Ke,inv(dcgain(sys_closed))*Be,Ce,sysddr.d,Ts2);
Gff=inv(dcgain(Sys_StateD));
% disturbance rejection rt=0 dr=1
% give initial condition
x0 = [0.001 0.001 0.001 1];
o0=  [0 0 0 0];
t3 = 0:Ts2:2;  % simulation time = 1 seconds
% Unit step signal 
u=1;
u(1:length(t3))=1; 
u(2:length(t3))=0;    
[Y, Tsim, X] = lsim(SysObsLoadD,u,t3,[x0 o0]); 
[Y1, Tsim1, X1] = lsim(Sys_StateD,u,t3,x0); 
figure
% b=[Y(:,1),Y(:,2)];
stairs(Tsim,Y1(:,1));

sysddr=c2d(sysc,Ts2,'zoh');
poles=[0.98+0.0017i 0.98-0.0017i 0.98];%change to Ts2
feedgain=place(sysddr.A,sysddr.B,poles); % Gain K
sys_closed = ss(sysddr.A - sysddr.B*feedgain, sysddr.B, sysddr.C, sysddr.D, Ts2); % Kr
poles_d_ob=[0.9896	0.9891	0.9886];% Observer poles
% Define new state matrices for constant load dist 
Ae = [sysddr.A sysddr.B; 0 0 0 0];
Be = [sysddr.B; 0];
Ce = [sysddr.C 0];
De = 0;
% New Feedback gain 
Ke = [feedgain 0.05];
% Design new observer gain
Le = place(Ae',Ce',[poles_d_ob 0.75])';
% Combine
At = [Ae -Be*Ke;Le*Ce Ae-Be*Ke-Le*Ce]; 
Bt = [Be*inv(dcgain(sys_closed));Be*inv(dcgain(sys_closed))] ;
Ct = [Ce 0 0 0 0;0 0 0 0 Ce];
SysObsLoadD = ss(At,Bt,Ct,sysddr.d,Ts2);
Sys_StateD = ss(Ae-Be*Ke,inv(dcgain(sys_closed))*Be,Ce,sysddr.d,Ts2);
Gff=inv(dcgain(Sys_StateD));
% disturbance rejection rt=0 dr=1
% give initial condition
x0 = [0.001 0.001 0.001 1];
o0=  [0 0 0 0];
t3 = 0:Ts2:2;  % simulation time = 1 seconds
% Unit step signal 
u=1;
u(1:length(t3))=1; 
u(2:length(t3))=0;    
[Y2, Tsim, X] = lsim(SysObsLoadD,u,t3,[x0 o0]); 
[Y3, Tsim1, X1] = lsim(Sys_StateD,u,t3,x0); 
figure
% b=[Y(:,1),Y(:,2)];
stairs(Tsim,Y(:,1));
hold on 
stairs(Tsim,Y3(:,1));
xlabel('time [s]')
ylabel ( 'Amplitide')
legend({'time delayed system','Original system'})


% sysddr=c2d(sysc,Ts2,'zoh');
% poles=[0.9984+0.0017i 0.9984-0.0017i 0.99];%change to Ts2
% feedgain=place(sysddr.A,sysddr.B,poles); % Gain K
% sys_closed = ss(sysddr.A - sysddr.B*feedgain, sysddr.B, sysddr.C, sysddr.D, Ts2); % Kr
% poles_d_ob=[0.9896	0.9891	0.9886];% Observer poles
% % Define new state matrices for constant load dist 
% Ae = [sysddr.A sysddr.B; 0 0 0 0];
% Be = [sysddr.B; 0];
% Ce = [sysddr.C 0];
% De = 0;
% % New Feedback gain 
% Ke = [feedgain 0.05];
% % Design new observer gain
% Le = place(Ae',Ce',[poles_d_ob 0.75])';
% % Combine
% At = [Ae -Be*Ke zeros(4,1);Le*Ce Ae-Be*Ke-Le*Ce zeros(4,1);zeros(1,3) -feedgain zeros(1,3)]; 
% Bt = [Be*inv(dcgain(sys_closed));Be*inv(dcgain(sys_closed));inv(dcgain(sys_closed))] ;
% Ct = [Ce 0 0 0 0 0;0 0 0 0 0 Ce];
% 
% poles=[0.9984+0.0017i 0.9984-0.0017i 0.95 0.9 0.85 0.8 0.75 0.7 0.65 ];%change to Ts2
% feedgain=place(At,Bt,poles); % Gain K
% 
% At = [Ae -Be*Ke zeros(4,1);Le*Ce Ae-Be*Ke-Le*Ce zeros(4,1);zeros(1,3) -feedgain zeros(1,3)]; 
% Bt = [Be*inv(dcgain(sys_closed));Be*inv(dcgain(sys_closed));inv(dcgain(sys_closed))] ;
% Ct = [Ce 0 0 0 0 0;0 0 0 0 0 Ce];
% 
% SysObsLoadD = ss(At,Bt,Ct,sysddr.d,Ts2);
% Sys_StateD = ss(Ae-Be*Ke,inv(dcgain(sys_closed))*Be,Ce,sysddr.d,Ts2);
% Gff=inv(dcgain(Sys_StateD));
% % disturbance rejection rt=0 dr=1
% % give initial condition
% x0 = [0.001 0.001 0.001 1 0.001];
% o0=  [0 0 0 0];
% t3 = 0:Ts2:2;  % simulation time = 1 seconds
% % Unit step signal 
% u=1;
% u(1:length(t3))=1; 
% u(2:length(t3))=0;    
% [Y3, Tsim, X] = lsim(SysObsLoadD,u,t3,[x0 o0]); 
% [Y1, Tsim1, X1] = lsim(Sys_StateD,u,t3,x0(1:4)); 
% figure
% % b=[Y(:,1),Y(:,2)];
% stairs(Tsim,Y(:,1));
% hold on
% stairs(Tsim,Y3(:,1));
% 
% u1=1;
% u1(1:length(t3))=1;
% xlabel('time [s]')
% ylabel ( 'Amplitide')
% legend({'System Output','Observer Output'})
% title('System Output vs Observer Output');
%% time delay for lQ controller
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
mq = [0.01,6500,0.01];
mr = 1;
Q = diag(mq); % Q=diag[1,1,100]
R = mr;% R=0.01
% Continous LQ control 
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = plantd.a;
B = plantd.b;
C = plantd.c; 
D = plantd.d;

Q1 = Q;
R1 = R;
% dlqr
[Kd2,S,e] = dlqr(A,B,Q1,R1,0);
GgainD2 = 1/((C-D*Kd2)*inv(eye(3)-(A-B*Kd2))*B+D);
SysD2 = ss(A-B*Kd2,GgainD2*B,C,0,h/2);

% Plot figure
figure
step (SysD2)
hold on
legend('previous controller','redesigned controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 1.1])


%time delay
[A,B,C,D]=tf2ss(num,den);
sysc=ss(A,B,C,D);
A = sysc.a; B = sysc.b; C = sysc.c; D = sysc.d;
sysc=ss(A,B,C,D);
mq = [0.01,10000,0.01];
mr = 1;
Q = diag(mq); 
R = mr;
[Kc,S,e] = lqr(sysc,Q,R,0);
GgainC = 1/((D-(C-(D*Kc)))*inv(A-(B*Kc))*B);
SysC = ss(A-B*Kc,GgainC*B,C,0);
% Discretize with appropriate sampling time 
[y,t] = step(SysC);
tmp = stepinfo(y,t)
Nr = 15;
h = tmp.RiseTime/Nr;
plantd = c2d(sysc ,h, 'zoh'); % Disc equiv


A = [plantd.a plantd.b;-Kc -10];
B = [plantd.b;GgainC];
C = [plantd.c 0]; 
D = plantd.d;
mq = [0.01,6500,0.01,1];
Q1 = diag(mq);
R1 = R;
% dlqr
[Kd,S,e] = dlqr(A,B,Q1,R1,0);
GgainD = 1/((C-D*Kd)*inv(eye(4)-(A-B*Kd))*B+D);
SysD = ss(A-B*Kd,GgainD*B,C,0,h/2);

% Plot figure
step (SysD)
legend('previous controller','time delayed controller')
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
xlim([0 1])

[y1,t1,x1] = step(SysD); 
stepinfo(y1,t1)
figure
%control effort
plot(t1,-Kd*x1'+GgainD);
xlabel ( 'Time[s]')
ylabel ( 'Amplitude')
title('Control Input of LQ controller')
xlim([0 2])