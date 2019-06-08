clear;
close all;

Path='20190606/test3/';
load(strcat(Path,'data.mat'));

m=size(controller.alpha,2);


meanx=351080;
meany=3433880;
figure(1); 
title('measurement')
subplot(611);
plot(estimator.timestamp, estimator.measurement(:,1)-meanx, '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp, estimator.state(:,1)-meanx, ':k', 'linewidth', 2);hold on;
plot(gps.timestamp, gps.x-meanx, '--b', 'linewidth', 2);
ylabel('x (m)');
legend('measurement', 'state','gps');
subplot(612); 
plot(estimator.timestamp, estimator.measurement(:,2)-meany, '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp, estimator.state(:,2)-meany, ':k', 'linewidth', 2);
plot(gps.timestamp, gps.y-meany, '--b', 'linewidth', 2);
ylabel('y (m)');
legend('measurement', 'state','gps');
subplot(613);
plot(estimator.timestamp, estimator.measurement(:,3)*180/pi, '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp, estimator.state(:,3)*180/pi, ':k', 'linewidth', 2); hold on;
plot(gps.timestamp, restrictheading(gps.heading), '--b', 'linewidth', 2);
ylabel('theta (deg)');
legend('measurement', 'state','gps');
subplot(614);
plot(estimator.timestamp, estimator.measurement(:,4), '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp, estimator.state(:,4), ':k', 'linewidth', 2);hold on;
plot(gps.timestamp, gps.u, '--b', 'linewidth', 2);
ylabel('u (m/s)');
legend('measurement', 'state','gps');
subplot(615);
plot(estimator.timestamp, estimator.measurement(:,5), '-r', 'linewidth', 2);hold on;
plot(estimator.timestamp, estimator.state(:,5), ':k', 'linewidth', 2);hold on;
plot(gps.timestamp, gps.v, '--b', 'linewidth', 2);
ylabel('v (m/s)');
legend('measurement', 'state','gps');
subplot(616);
plot(estimator.timestamp, estimator.measurement(:,6), '-r', 'linewidth', 2);
hold on;
plot(estimator.timestamp, estimator.state(:,6), ':k', 'linewidth', 2);
ylabel('r (rad/s)');
legend('measurement', 'state');

    
figure(2);
for i=1:m
    subplot(m,1,i);
    plot(controller.timestamp, controller.alpha(:,i),'linewidth', 2); 
    ylabel(['alpha(deg) ',num2str(i)]);
end

figure(3);
for i=1:m
    subplot(m,1,i);
    plot(controller.timestamp, controller.rpm(:,i),'linewidth', 2); 
    ylabel(['rotation(rpm) ',num2str(i)]);
end



function head=restrictheading(heading)
    if heading>180
        head=heading-360;
    elseif heading<-180
        head=heading+360;
    else
        head =heading;  
    end
    
end

