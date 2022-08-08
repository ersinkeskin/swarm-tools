iters = [200 350 750]
% iters = [200 350 550]

h = figure;
h.Position = [0 0 1200 500]
h.PaperPositionMode = 'auto'
for i = 1:3
    subplot(1,3,i),hold all
    a = gca;
    hold all

    for j = 1:size(obsList,1)
        viscircles(obsList(j,[1,2]),obsList(j,3),'EdgeColor','r');
    end
    
    i2 = iters(i);
    l = plot(posx_log(i2,:),posy_log(i2,:),'o','linewidth',1.5,'markersize',8);
    l2 = quiver(posx_log(i2,:),posy_log(i2,:),velx_log(i2,:),vely_log(i2,:),0,'linewidth',1.5);
    ylim([-1 1]*(sideLength-1)*sqrt(3)*1.3/2)
    xlim([-1 1]*(sideLength-1)*sqrt(3)*1.3/2 + mean(posx_log(i2,:)))
        daspect([1 1 1]);
        grid
        title(['iteration = ',num2str(i2)])
end

tightfig

%%
print('KalmanSnapshot_b10.eps','-depsc')

%%

iters = [1 500 999];

h = figure;
    hold all
h.Position = [0 0 900 500];
    for j = 1:size(obsList,1)
        viscircles(obsList(j,[1,2]),obsList(j,3),'EdgeColor','r');
    end
for i = 1:3
%     subplot(1,3,i),hold all
%     a = gca;



    
    i2 = iters(i);
    l = plot(posx_log(i2,:),posy_log(i2,:),'o');
    l2 = quiver(posx_log(i2,:),posy_log(i2,:),velx_log(i2,:),vely_log(i2,:),0);

end

    ylim([-1 1]*(sideLength-1)*sqrt(3)*1.3/2)
%     xlim([-1 1]*(sideLength-1)*sqrt(3)*1.3/2 + mean(posx_log(i2,:)))
        daspect([1 1 1]);
        grid