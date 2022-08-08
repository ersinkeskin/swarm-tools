%%
figure,hold all
plot(xrArr(1,:),xrArr(2,:),'o')
plot(measArr(1,:),measArr(2,:),'x')
plot(xkArr(1,:),xkArr(2,:),'d')

%% hiz
figure,hold all
plot( sqrt(diff(measArr(1,:)).^2 + diff(measArr(2,:)).^2)/ts)
plot(xkArr(3,:))
    plot(xrArr(3,:))
    plot(xpreArr(3,:))
    legend meas xk xr xpre
    grid on
    
%% process controllers
figure, hold all
plot(velApp_log(:,iFocal))
plot(velCom_log(:,iFocal))

figure, hold all
plot(ortApp_log(:,iFocal))
plot(ort_log(:,iFocal))
%%
figure;
for i = 1:n
    subplot(n,1,i);hold all
    i = i + n*2;
    if(i==1 || i==2  )
        plot(xrArr(i,:)-measArr(i,:),'o')
%     elseif i == 4
%         plot(xrArr(i,:)-measArr(i-1,:),'o')
    end
    
    plot(xrArr(i,:)-xkArr(i,:),'x')
    plot((sqrt(Parr(i,:))),'color','red','LineStyle','--')
    plot(-(sqrt(Parr(i,:))),'color','red','LineStyle','--')
    grid
end

%%
for i =1:n
    i = i + n*1;
    figure,hold all
    if (mod(i,4) == 0 && i~=4)
%         xrArr(i,:) = wrapToPi(xrArr(i,:));
    end
    plot(xrArr(i,:))
    plot(xkArr(i,:))
%     plot(xpreArr(i,:))
    title(num2str(i))
end

%% states
figure;
idx = 1;
for ir = 1:nr
    i = idx + (ir-1)*n;
    subplot(nr,1,ir);hold all
    plot(xrArr(i,:)-xkArr(i,:),'x')
    %     if(i==1 || i==2 || i==6 || i==7 )
    %         plot(xrArr(i,:)-measArr(i,:),'o')
    %     end
    plot((sqrt(Parr(i,:))),'color','red','LineStyle','--')
    plot(-(sqrt(Parr(i,:))),'color','red','LineStyle','--')
    grid
end

%%
figure,hold all
    plot(xrArr(4,:))
    plot(xkArr(4,:))
    plot(xpreArr(4,:))
    plot(measArr(3,:))
    legend gercek xk xpre meas
    
%%
rmsVal = rms(xrArr(3,:)-xkArr(3,:))
maxVal = max(xrArr(3,:)-xkArr(3,:))