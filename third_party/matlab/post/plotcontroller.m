clear;
close all;

Path='20190606/test3/';
load(strcat(Path,'data.mat'));


index_actuation=0; % underactuated=1


m=size(controller.alpha,2);

figure(1); 
subplot(311);
plot(controller.timestamp, controller.est(:,1), '-r', 'linewidth', 2);
hold on;
plot(controller.timestamp, controller.tau(:,1), ':k', 'linewidth', 2);
ylabel('taux(N)');
legend('estimated force', 'desired force');
subplot(312); 
plot(controller.timestamp, controller.est(:,2),'-r', 'linewidth', 2);
hold on;
plot(controller.timestamp, controller.tau(:,2), ':k', 'linewidth', 2);
ylabel('tauy(N)') 
legend('estimated force', 'desired force') 
subplot(313);
plot(controller.timestamp, controller.est(:,3), '-r', 'linewidth', 2);
hold on;
plot(controller.timestamp, controller.tau(:,3), ':k', 'linewidth', 2);
legend('estimated force', 'desired force');
ylabel('taun(N m)')

    
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

