clear;
close all;

wp0=[2;0.1];
wp1=[-5;2];

averagelength=0.2;
delta=wp1-wp0;
L=sqrt(delta(1)*delta(1)+delta(2)*delta(2));
r=L/2;
thetaK=atan(delta(2)/delta(1));

if(delta(1)<0)
    thetaK=thetaK+pi;
end

gamma = acos(L/(2*r));
    

center1=wp0 + r*[cos(thetaK+gamma);sin(thetaK+gamma)];
center2=wp0 + r*[cos(thetaK-gamma);sin(thetaK-gamma)];

thetaset1=linspace(thetaK+gamma-pi, thetaK-gamma, 20);
thetaset2=linspace(thetaK+gamma, thetaK-gamma+pi, 20);
for i=1:20
    pointset1(:,i)=center1+r*[cos(thetaset1(i));sin(thetaset1(i))];
    pointset2(:,i)=center2+r*[cos(thetaset2(i));sin(thetaset2(i))];
end

figure(1)
plot(wp0(2),wp0(1), 'r*');hold on;
plot(wp1(2),wp1(1), 'r*');hold on;
plot(center1(2),center1(1), 'k*');hold on;
plot(center2(2),center2(1), 'b*');hold on;
for i=1:20
    plot(pointset1(2,i),pointset1(1,i), 'sk');hold on;
    plot(pointset2(2,i),pointset2(1,i), 'sb');hold on;
end
grid on;
axis equal;
