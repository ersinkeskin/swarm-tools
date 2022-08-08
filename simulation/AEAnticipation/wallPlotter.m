figure(100)
h = figure(100);
h.Position = [100 100 1000 500];
clf
% a = axes;
a1 = subplot(1,2,1);
hold all
daspect([1 1 1]);
a1.Position = [0.0300    0.15    0.5    0.75];
a1.FontSize = 12;



% subplot(1,2,1);
switch obsType
    case 1
        for i = 1:size(obsList,1)
            viscircles(obsList(i,[1,2]),obsList(i,3),'EdgeColor','r');
        end
        xlim([-5*1 30])
    case 2
        line([1 1]*obsList(1,1),[-10 10],'color','r','linewidth',2)
        xlim([-8 12])
end

l = plot(posx_log(1,:),posy_log(1,:),'o');
l2 = quiver(posx_log(1,:),posy_log(1,:),velx_log(1,:),vely_log(1,:));

a2 = subplot(1,2,2);
a2.Position = [0.600    0.2    0.1    0.55];
a2.FontSize = 12;
grid on

% xlim([-10 10])
% ylim([-10 10])
pause(1)
for i = 1:250:size(posx_log,1)
    figure(100)
    l.XData = posx_log(i,:);
    l.YData = posy_log(i,:);
    %
    l2.XData = posx_log(i,:);
    l2.YData = posy_log(i,:);
    l2.UData = velx_log(i,:);
    l2.VData = vely_log(i,:);
    
%     title(['b=',num2str(b),' iter=',num2str(i)])
%     title(['iter=',num2str(i)])
%     xlim([-5*1 30])
    xlim([-8 12])
    
    
    a2 = subplot(1,2,2);
    a2.Position = [0.600    0.15    0.3    0.75];
    xlim([0 5e4])
    ylim([0 1])
    cla
    hold all
    plot(psi(1:i),'linewidth',1.5)
    plot(i,psi(i),'o','linewidth',1.5)
    title('Order Metric ($\psi$)','interpreter','latex')
    grid on
    
    drawnow;
    %         pause(0.1);
end

hold off

% figure(101),hold all
% plot(psi)
% figure(102),hold all
% plot(psi2)
