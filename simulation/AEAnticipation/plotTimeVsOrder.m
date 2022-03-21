etaArr = [40 43 46 48];
stIdx = 5e5;
% etaArr = [40:60:220];
% stIdx = 5e4;

h = figure;
h.Position= [0 0 800 600];

for i = 1:4
    a = subplot(2,2,i);
    hold all
    dataF = data_all.n32.p0.(['Eta',num2str(etaArr(i))]).psi';
    [val idx] = (max(dataF(stIdx:end,:)));
    [val2 idx2] = max(val);
    plot(idx(idx2)+stIdx-1,dataF(idx(idx2)+stIdx-1,idx2),'o','markersize',10,'linewidth',2,'color','r')
    plot(dataF)
    line([1 1]*stIdx,[0 1],'linestyle','--','color','black')
    xlabel('Iteration')
    ylabel('\psi')
    a.FontSize = 10;
    xlim([0 size(dataF,1)])
    ylim([0 1])
    title(['\eta = ',num2str(etaArr(i)),'°'])
    
end
tightfig

%%
% print('AE_orderTimeseries','-depsc')