strc = data_all.n32;

startIdx = 4e5;

p_names = fieldnames(strc);

lower_modes = zeros(6,1);
upper_modes = zeros(6,1);

figure(1)
hold all
for i = 1:length(p_names)
    strc_etas = strc.(cell2mat(p_names(i)));
    
    eta_names = fieldnames(strc_etas);
    
    eta_arr = [];
    mode_arr  = [];
    colStr = {'y' 'm' 'c' 'r' 'g' 'b'};
    for j = 1:length(eta_names)
        temp1 = cell2mat(eta_names(j));
        eta_arr = [eta_arr str2num(temp1(4:end))];
        psi_raw = strc_etas.(temp1).psi;
%         modes = mode(round(1e3*strc_etas.(temp1).psi),2)/1e3;
%         modes = mean(strc_etas.(temp1).psi,2);
        modes = (max(strc_etas.(temp1).psi(:,startIdx:end),[],2));
        mode_arr = [mode_arr max(modes)];
%         mode_arr = [mode_arr mean(modes(modes>0.5))];
        
        figure(2)
%         subplot(2,3,i)
        hold all
        plot(str2num(temp1(4:end)),modes,'o')
%         lower_modes(i) = lower_modes(i)+sum(modes<0.5);
%         upper_modes(i) = upper_modes(i)+sum(modes>=0.5);
        
        modes_up = modes;
        modes_up(modes<0.5) = NaN;
%         modes_up_arr = [modes_up_arr; modes_up];
            
%     figure(3)
%     hold all
%     plot(str2num(temp1(4:end)),mean(modes_up(~isnan(modes_up))),'color',colStr{i},'marker','o','linestyle','none')
%     grid on
              
    end
    
    [val,idx] = sort(eta_arr);
    
    figure(1)
    plot(val,mode_arr(idx),'marker','o','linewidth',1)
    xlabel('\eta (°)')
    ylabel('\psi')
       
end
grid, grid minor

legend_str = {'p=0' ,'p=0.2', 'p=0.4', 'p=0.6', 'p=0.8', 'p=1'};
% legend_str = {'p=0' ,'p=0.2',  'p=0.6',  'p=1'};
lg = legend(legend_str);
lg.FontSize = 14;

% figure(3)
% legend_str = {'p=0' ,'p=0.2', 'p=0.4', 'p=0.6', 'p=0.8', 'p=1'};
% lg = legend(legend_str);
% lg.FontSize = 14;

figure(2)
for i = 1:6
%     subplot(2,3,i)
%     xlabel([legend_str{i},' lm: ',num2str(lower_modes(i)),' um: ',num2str(upper_modes(i))])
    xlabel([legend_str{i}])
    %     annotation('textbox','string',['lm: ',num2str(lower_modes(i)),' um: ',num2str(upper_modes(i))])
    grid, grid minor
end
