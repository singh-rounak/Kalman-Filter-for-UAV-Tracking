# Kalman-Filter-for-UAV-Tracking
Original Kalman Filter (using MATLAB) to track UAVs in 3D space.


clc; close all;

%initial state
xo=25;
vox=2;
%Observations ****

% X-Direction Calculation:
X = xlsread('DroneFlightTrace1.xlsx','R2:R12000');
V = 0.1 .*[X]+1;
%Process Errors in Process Covaiance Matrix
del_px=20; %initial covariance matrix is choosen intuitively
del_pv=5;
%initial conditions
acc_x=2;
del_t=1;
vx=2;
del_x=25; %uncertainity in the measurement
%Observation Error
del_X=25;
del_VX=6;
Xk=[];
%The Predicted State
A=[1 del_t;0 1];
B=[(0.5*((del_t).^2));del_t];
p=[];
for k=1:1:length(X)-1;
Xk_= [X(k);V(k)];
uk=acc_x;
Xkp1=((A*Xk_));
Xkp2=((B*uk));   
Xkp=(Xkp1+Xkp2);%this is our first estimation
p=[p;Xkp];
%Initialising Process Covariance Matrix
Pk_=[((del_px).^2) 0;0 ((del_pv).^2)];
%Predicted Process Covariance Matrix
Pkp1=((A)*(Pk_));
Pkp2=((Pkp1)*(A'));
pkp=(Pkp2-[0 Pkp2(2);Pkp2(3) 0]); %since the 2nd and 3rd term are not imp.
%Calculating the Kalman gain
R=[((del_X)^2) 0;0 ((del_VX)^2)];
H=[1 0 ; 0 1];
K=((pkp)*H')/((H*pkp*H')+R);
%The New Observation
k=k+1;
Ykm=[X(k);V(k)];
C=[1 0;0 1];
Yk=C*Ykm;
%Calculating the Current State
Xk=[Xk; Xkp + K*(Yk-(H*(Xkp)))];
%Updating the process covariance matrix
Pk1=((eye)-(K*H))*pkp;
pk=(Pk1-[0 Pk1(3);Pk1(2) 0]);
k=k+1;
end
Xkf=[X(1)];
Vkf=[V(1)];
for k=1:2:(length(Xk)-1)
    Xkf=[Xkf;Xk(k)];
end
for k=2:2:(length(Xk))
    Vkf=[Vkf;Xk(k)];
end
prx=[X(1)];
prv=[V(1)];
for i=1:2:(length(p)-1)
    prx=[prx;p(i)];
end
for i=2:2:(length(p))
    prv=[prv;p(i)];
end

%%% Y-D Calculations:
Y1 = xlsread('DroneFlightTrace1.xlsx','S2:S12000');
V1 = 0.1.*[Y1];
%Process Errors in Process Covaiance Matrix
del_py=20; %initial covariance matrix is choosen intuitively
del_pv1=5;
%initial conditions
acc_x=2;
acc_y=2;
del_t=1;
vy=2;
del_y=25; %uncertainity in the measurement
%Observation Error
del_Y=25;
del_VY=6;
Yk=[];
%The Predicted State
A=[1 del_t;0 1];
fprintf A;
B=[(0.5*((del_t).^2));del_t];
fprintf B;
q=[];
fprintf q;
for k=1:1:length(Y1)-1;
Yk_= [Y1(k);V1(k)];
uk2=acc_x;
Ykp1=((A*Yk_));
Ykp2=((B*uk2));   
Ykp=(Ykp1+Ykp2);%this is our first estimation
q=[q;Ykp];
%Initialising Process Covariance Matrix
Pk2_=[((del_py).^2) 0;0 ((del_pv1).^2)];
%Predicted Process Covariance Matrix
Pkp12=((A)*(Pk2_));
Pkp22=((Pkp12)*(A'));
pkp2=(Pkp22-[0 Pkp22(2);Pkp22(3) 0]); %since the 2nd and 3rd term are not imp.
%Calculating the Kalman gain
R1=[((del_Y)^2) 0;0 ((del_VY)^2)];
H=[1 0 ; 0 1];
K=((pkp2)*H')/((H*pkp2*H')+R1);
%The New Observation
k=k+1;
Ykm1=[Y1(k);V1(k)];
C=[1 0;0 1];
Yk1=C*Ykm1;
%Calculating the Current State
Yk=[Yk; Ykp + K*(Yk1-(H*(Ykp)))];
%Updating the process covariance matrix
Pk12=((eye)-(K*H))*pkp2;
pk2=(Pk12-[0 Pk12(3);Pk12(2) 0]);
k=k+1;
end
Ykf=[Y1(1)];
Vkf1=[V1(1)];
for k=1:2:(length(Yk)-1)
    Ykf=[Ykf;Yk(k)];
end
for k=2:2:(length(Yk))
    Vkf1=[Vkf1;Yk(k)];
end
pry=[Y1(1)];
prv1=[V1(1)];
for i=1:2:(length(q)-1)
    pry=[pry;q(i)];
end
for i=2:2:(length(q))
    prv1=[prv1;q(i)];
end
%%%% Z-D Calculations:
Z = xlsread('DroneFlightTrace1.xlsx','T2:T12000');
V3 = 0.1.*[Z];
%Pr ocess Errors in Process Covaiance Matrix
del_pz=10; %initial covariance matrix is choosen intuitively
del_pv3=5;
%initial conditions
acc_z=2;
del_t=1;
v3z=2;
del_z=12; %uncertainity in the measurement
%Observation Error
del_Z=25;
del_VZ=6;
Zk=[];
%The Predicted State
A=[1 del_t;0 1];
B=[(0.5*((del_t).^2));del_t];
r=[];
for k=1:1:length(Z)-1;
Zk_= [Z(k);V3(k)];
uk3=acc_z;
Zkp1=((A*Zk_));
Zkp2=((B*uk3));   
Zkp=(Zkp1+Zkp2);%this is our first estimation
r=[r;Zkp];
%Initialising Process Covariance Matrix
Pk3_=[((del_pz).^2) 0;0 ((del_pv3).^2)];
%Predicted Process Covariance Matrix
Pkp3=((A)*(Pk3_));
Pkp4=((Pkp3)*(A'));
pkp5=(Pkp4-[0 Pkp4(2);Pkp4(3) 0]); %since the 2nd and 3rd term are not imp.
%Calculating the Kalman gain
R3=[((del_Z)^2) 0;0 ((del_VZ)^2)];
H2=[1 0 ; 0 1];
K=((pkp5)*H2')/((H2*pkp5*H2')+R3);
%The New Observation
k=k+1;
Zkm=[Z(k);V3(k)];
C=[1 0;0 1];
Zk1=C*Zkm;
%Calculating the Current State
Zk=[Zk; Zkp + K*(Zk1-(H2*(Zkp)))];
%Updating the process covariance matrix
Pkp3=((eye)-(K*H2))*pkp5;
pk6=(Pkp3-[0 Pkp3(3);Pkp3(2) 0]);
k=k+1;
end
Zkf=[Z(1)];
V3kf=[V3(1)];
for k=1:2:(length(Zk)-1)
    Zkf=[Zkf;Zk(k)];
end
for k=2:2:(length(Zk))
    V3kf=[V3kf;Zk(k)];
end
prZ=[Z(1)];
prv3=[V3(1)];
for i=1:2:(length(r)-1)
    prZ=[prZ;r(i)];
end
for i=2:2:(length(r))  
    prv3=[prv3;r(i)];
end

E=[];
for i=1:1:(length(X))
    E1=abs(((Xkf(i)-X(i))/Xkf(i))*100);
end
for i=1:1:(length(Y1))
    E2=abs(((Ykf(i)-Y1(i))/Ykf(i))*100);
end
for i=1:1:(length(Z))
    E3=abs(((Zkf(i)-Z(i))/Zkf(i))*100);
end
E = [E,E1,E2,E3]


%Plotting data

plot3(Xkf,Ykf,Zkf,'r--x');hold on;
plot3(prx,pry,prZ,'b--x');hold off;
grid on;
xlabel('position in x direction (m)');
ylabel('position in y direction (m)');
zlabel('position in z direction (m)');
legend('Actual position','estimation','predicted');
title('Position estimation of the drone');

% 
% plot(E,'r--x');




