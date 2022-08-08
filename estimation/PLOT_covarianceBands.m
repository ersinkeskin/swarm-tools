t = (1:iterLim)*ts;
h = figure;
h.Position = [0 0 1200 1200];
h.PaperPositionMode  ='auto';


for i = 1:n
    a = subplot(n,1,i);hold all
    
    if i == 4
        plot(t,(xrArr(i,:)-xkArr(i,:))*180/pi,'x','color','b')
        plot(t,(sqrt(Parr(i,:)))*180/pi,'color','r','LineStyle','--')
        plot(t,-(sqrt(Parr(i,:)))*180/pi,'color','r','LineStyle','--')
    else
        plot(t,xrArr(i,:)-xkArr(i,:),'x','color','b')
        plot(t,(sqrt(Parr(i,:))),'color','r','LineStyle','--')
        plot(t,-(sqrt(Parr(i,:))),'color','r','LineStyle','--')
    end
    
    grid on
    a.FontSize=12;
    switch i
        case 1
            ylabel('$$x_1 - \hat{x}_1 (m)$$','Interpreter','Latex','FontSize',16)
        case 2
            ylabel('$$y_1 - \hat{y}_1 (m)$$','Interpreter','Latex','FontSize',16)
        case 3
            ylabel('$$v_1 - \hat{v}_1 (m/s)$$','Interpreter','Latex','FontSize',16)
        case 4
            ylabel('$$\theta_1 - \hat{\theta}_1 (^{\circ})$$','Interpreter','Latex','FontSize',16)
            xlabel('Time (s)')
    end
end

tightfig