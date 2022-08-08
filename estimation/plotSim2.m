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

%     viscircles(obsList(1,[1,2]),obsList(1,3)-obsD,'EdgeColor','g');
%     rectangle('Position',[-obsList(1,3) -obsList(1,3) obsList(1,3)*2 obsList(1,3)*2])
%     rectangle('Position',[-(obsList(1,3)+0.5) -(obsList(1,3)+0.5) (obsList(1,3)+0.5)*2 (obsList(1,3)+0.5)*2],'EdgeColor','r','linewidth',2)
    
l = plot(posx_log(1,:),posy_log(1,:),'o');
l2 = quiver(posx_log(1,:),posy_log(1,:),velx_log(1,:),vely_log(1,:),0);

xlim([0 6])
ylim([-1 1]*(sideLength-1)*sqrt(3)*1.3/2)
pause(2)
ffw = 3;
step= 1;
sleepDuration = ts*1000/ffw;
for i = 1:step:size(posx_log,1)
    l.XData = posx_log(i,:);
    l.YData = posy_log(i,:);
    %
    l2.XData = posx_log(i,:);
    l2.YData = posy_log(i,:);
    l2.UData = velx_log(i,:);
    l2.VData = vely_log(i,:);

    title(['b=00, t=',num2str(i*ts),' iter: ',num2str(i)])
%     ylim([-10 10])
%     xlim([0 35])
    drawnow;
    java.lang.Thread.sleep(sleepDuration);
end

hold off