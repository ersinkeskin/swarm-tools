figure(100)
h = figure(100);
h.Position = [100 100 1000 500];
clf
a = axes;
hold all
daspect([1 1 1]);
    for i = 1:size(obsList,1)
        viscircles(obsList(i,[1,2]),obsList(i,3),'EdgeColor','r');
    end

l = plot(posx_log(1,:),posy_log(1,:),'o');
l2 = quiver(posx_log(1,:),posy_log(1,:),velx_log(1,:),vely_log(1,:));

% xlim([-10 30])
% ylim([-obsY obsY])
pause(2)
for i = 1:300:size(posx_log,1)
    l.XData = posx_log(i,:);
    l.YData = posy_log(i,:);
    %
    l2.XData = posx_log(i,:);
    l2.YData = posy_log(i,:);
    l2.UData = velx_log(i,:);
    l2.VData = vely_log(i,:);

    title(['b=00, iter=',num2str(i)])
    drawnow;
%         pause(0.1);
end

hold off