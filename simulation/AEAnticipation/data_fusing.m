% data_str = 'data_piece73847259436.mat';
load(data_str)
% load data_all
% save data_all_1stepback data_all
data_all.n32 = [];
data_all.fuse_log = [];
%%
% if (data_piece.nAgents == 1024)
    strc = data_all.n32;
% else
%     if(data_piece.nAgents == 100^2)
%         strc = data_all.n100;
%     end
%     if(data_piece.nAgents == 300^2)
%         strc = data_all.n300;
%     end
%     if(data_piece.nAgents == 16^2)
%         strc = data_all.n16;
%     end
% end
%%

for i = 1:length(data_piece.p_sweep)
    pStr = ['p',num2str(round(data_piece.p_sweep(i)*1000))];
    if(~isfield(strc,pStr))
        strc.(pStr) = [];
    end
    
    for j = 1:length(data_piece.Eta_sweep)
    EtaStr = ['Eta',num2str(data_piece.Eta_sweep(j))];
        if(~isfield(strc.(pStr), EtaStr))
        strc.(pStr).(EtaStr) = [];
        end
        
        if(~isfield(strc.(pStr).(EtaStr),'psi'))
            strc.(pStr).(EtaStr).psi = [];
            strc.(pStr).(EtaStr).orts = [];
        end
        
        strc.(pStr).(EtaStr).psi = [strc.(pStr).(EtaStr).psi; squeeze(data_piece.psi(i,j,:,:))];
%         strc.(pStr).(EtaStr).orts = [strc.(pStr).(EtaStr).orts; squeeze(data_piece.orts(i,j,:,:))];
    end
end

% if (data_piece.nAgents == 1024)
    data_all.n32=strc;
% else
%     if(data_piece.nAgents == 100^2)
%         data_all.n100 = strc;
%     end
%         if(data_piece.nAgents == 300^2)
%         data_all.n300 = strc;
%         end
%         if(data_piece.nAgents == 16^2)
%         data_all.n16 = strc;
%     end
% end
str_idx = length(data_all.fuse_log)+1;
% if(str_idx==2)
%     str_idx=1;
% end
data_all.fuse_log{str_idx} = data_str;
save data_all data_all

%%

% data_all.fuse_log = cell(1);
% data_all.n32 = [];
% data_all.n100 = [];
% save data_all data_all