% function to perform los algorithm 

close all;
clear;

sampletime=0.1;
% mass and damping matrix
M=[25.8 0 0;
    0 33.8 1;
    0 1 2.76];
D=[2 0 0;
    0 7 0.1;
    0 0.1 0.5];


L=1.255;


% waypoints
wpt=[0.372 -0.181;
    -0.628 1.320;
    0.372 2.820;
    1.872 3.320;
    6.872 -0.681;
    8.372 -0.181;
    9.372 1.320;
    8.372 2.820
    ];

% capture radius
captureradius=0.8*L;

ud=0.1; 

% initial condition
x0=-0.2;
y0=0;
theta0=1.78;
u0=0.1;
v0=0;
r0=0;




totalpnum=size(wpt,1);

figure(1)
for i=1:(totalpnum-1)
    plot(wpt(i,2), wpt(i,1),'r*');hold on;
    plot([wpt(i,2) wpt(i+1,2)], [wpt(i,1) wpt(i+1,1)],':b'); hold on;
end
plot(wpt(totalpnum,2), wpt(totalpnum,1),'r*');hold on;

length=22000;
x=zeros(6,length);
x(:,1)=[x0;
        y0;
        theta0;
        u0;
        v0;
        r0];
tau=zeros(3,length);
dtheta=zeros(1,length);
thetaerror=zeros(1,length);
crosserror=zeros(1,length);
control=zeros(2,length);
integral_error=zeros(10,1);
index_wpt=2;
for i=1:length
    
    plot(x(2,i), x(1,i),'k.');hold on;
    if ((wpt(index_wpt,1)-x(1,i))^2+(wpt(index_wpt,2)-x(2,i))^2)< captureradius^2
        index_wpt=1+index_wpt;
    end
    if index_wpt>totalpnum
        break;
    end
    [dtheta(i), crosserror(i)]=computelospoint(L, wpt(index_wpt-1,1), ...
        wpt(index_wpt-1,2), wpt(index_wpt,1), wpt(index_wpt,2), x(1,i),x(2,i));
    thetaerror(i)=dtheta(i)- x(3,i);
    control(:,i+1)=computepid(x(:,i),ud, dtheta(i),integral_error, control(2,i));
    tau(:,i+1)=estimatethrust(control(2,i+1),control(1,i+1));
    if(abs(dtheta(i)-x(3,i))<pi/30)
        [A, B]=computeA(M,D, x(4:6), dtheta(i), sampletime);
    else
        [A, B]=computeA(M,D, x(4:6), x(3,i), sampletime);
    end
    x(:,i+1)=A*x(:,i)+B*tau(:,i+1);
end
xlabel('East[m]');
ylabel('North[m]');
axis equal;
grid on;


figure(2)
subplot(311)
title('position');
plot(x(1,:));
subplot(312)
plot(x(2,:));
subplot(313)
plot(dtheta,'-r'); hold on
plot(x(3,:),'--k');


figure(3)

subplot(311)
title('velocity');
plot(x(4,:));
subplot(312)
plot(x(5,:));
subplot(313)
plot(x(6,:));


% compute the damping matrix
function C=computeCb(m11,m22,m23,velocity)
C=zeros(3);
C(1,3)=-m22*velocity(2)-m23*velocity(3);
C(3,1)=m22*velocity(2)+m23*velocity(3);
C(2,3)=m11*velocity(1);
C(3,2)=-m11*velocity(1);
end

% compute the A matrix
function [A, B]=computeA(M,D, velocity, theta, sampletime)
% Cb=computeCb(M(1,1),M(2,2),M(2,3),velocity);
Cb=zeros(3,3);
M_inv=inv(M);
Ac=[zeros(3,3) computeJ(theta);
    zeros(3,3) -M_inv*(Cb+D)];
A=eye(6)+sampletime*Ac;
B=sampletime*[zeros(3,3);
               M_inv];

end


% compute the coodinate transform matrix
function J=computeJ(theta)
R=computeR(theta);
J=zeros(3,3);
J(1:2,1:2)=R;
J(3,3)=1;
end

% compute the coodinate transform matrix
function R=computeR(theta)
svalue=sin(theta);
cvalue=cos(theta);
R(1,1)=cvalue;
R(1,2)=-svalue;
R(2,1)=svalue;
R(2,2)=cvalue;
end

% compute the location of LOS point
function [thetalos,e]=computelospoint(r, xk0, yk0, xk, yk, x,y)
%% 坐标系转换
deltay=yk-yk0;
deltax=xk-xk0;
thetaK=atan(deltay/deltax);
if(deltax<0)
    thetaK=thetaK+pi;
end

R=computeR(thetaK);
eta=R'*[x-xk0; y-yk0];
e=eta(2);
if (e>r) % 没有交点
    thetar=-pi/2;
elseif (e<-r) % 没有交点
    thetar=pi/2;
else % 交点
    thetar=asin(-e/r);
end

thetalos=thetar+thetaK;
end

function tau=estimatethrust(alpha, rotation)
alpha=alpha*180/pi; % in degree
K=2e-5;
Teffective=K*rotation*rotation;
Cy=0.0126*1.1*0.9;
Cx=0.02*Cy;
Fsurge=Teffective*(1-Cx*alpha*alpha);
Fsway=Teffective*Cy*alpha;

tau(1)=Fsurge;
tau(2)=Fsway;
tau(3)=-0.8*Fsway;
end



function control=computepid(x,ud, dtheta,integrel_int, alpha_former)
Psurge=20;
Pyaw=10;
Dyaw=5;
Iyaw=0;


rotation=Psurge*(ud-x(4))+100;
if rotation>320
    rotation=320;
end

error_yaw=x(3)-dtheta;
integrel_int(1:(end-1))=integrel_int(2:end);
integrel_int(end)=(error_yaw);
alpha=Pyaw*error_yaw+Iyaw*sum(integrel_int)+Dyaw*x(6);

Max_alpha=pi/6;
Max_delta_alpha=0.004;

alpha_upper=min(Max_delta_alpha+alpha_former,Max_alpha);
alpha_lower=max(-Max_delta_alpha+alpha_former,-Max_alpha);

if alpha>alpha_upper
    alpha=alpha_upper;
end
if alpha<alpha_lower
    alpha=alpha_lower;
end

control(1)=rotation;
control(2)=alpha;
end
