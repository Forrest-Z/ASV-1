clear;
close all;

rpm=load('matlab.mat');


x = rpm.positive(:,1);
y = rpm.positive(:,2);
p = polyfit(x,y,2);
f = polyval(p,x);

x1 = (0:10:3000)';
f1 = polyval(p,x1);
fig=figure(1);
plot(x,y,'.','MarkerSize',15);
hold on;
plot(x1,f1,'-r','linewidth',1);
l1=legend('Raw Data','Quadraic Fit of raw data');
set(l1,'box','off');
xlabel('rotation (rpm)');
ylabel('Thrust (N)')
axis([0 3000 -2 20])
set(gca,'box','off');
set(gca,'TickDir','out');
text(300, 10.3, sprintf('$y=%.8fx^2%.8fx%.5f$',p), 'interpreter', 'latex', 'FontSize', 13)

% hold off

print(fig,'positive.jpg','-djpeg','-r300');