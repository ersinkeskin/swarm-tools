%%
% load hex_ms_20
load hex_ms_40

%generate_matrix_mex
% nAgents = mSize^2;
p_sweep = 0:0.2:1;
nofConfigs = 40;
%%
nLinks = length(m1);
rng('shuffle');

for i = 1:length(p_sweep)
    pLinks = round(nLinks*p_sweep(i));
    p_str = ['p_',num2str(p_sweep(i)*10)];
    
    for j = 1:nofConfigs
        while 1 %connectivity test.
            m1_temp = m1;
            m2_temp = m2;
            IL_temp = IL;
            if(i~=1)
                idx_rand = randperm(nLinks);
                m1_temp = m1_temp(idx_rand);
                m2_temp = m2_temp(idx_rand);
                IL_temp = IL_temp(idx_rand);
            end
            
            for k = pLinks:-1:1
                idx1 = randi(nAgents);
                while 1
                    idx2 = randi(nAgents);
                    flag = 1;
                    if(idx1==idx2)
                        flag=0;
                    else
                        %                     for l = (k+1):nLinks
                        %                         if(idx1==m1_temp(l))
                        %                             if(idx2==m2_temp(l))
                        %                                 flag=0;
                        %                                 break
                        %                             end
                        %                         end
                        %                     end
                        
                        if(k~=nLinks)
                            idx_m1 = find(m1_temp(k+1)==idx1);
                        else
                            
                            if(~isempty(idx_m1))
                                if(~isempty(find(m2_temp(idx_m1)==idx2,1,'first')))
                                    flag=0;
                                end
                            end
                        end
                    end
                    if(flag==1)
                        break
                    end
                    
                end
                
                m1_temp(k) = idx1;
                m2_temp(k) = idx2;
                
                IL_temp(k) = sqrt((posx(idx1)-posx(idx2))^2+(posy(idx1)-posy(idx2))^2);
            end
            
            conn_matrix_str.(p_str).m1{j} = m1_temp -1;
            conn_matrix_str.(p_str).m2{j} = m2_temp -1; %C notasyonu 0 based
            conn_matrix_str.(p_str).IL{j} = IL_temp;
            disp([i,j])
            
            conn_m = zeros(nAgents);
            for i2 = 1:length(m1)
                conn_m(m1_temp(i2),m2_temp(i2)) = 1;
                conn_m(m2_temp(i2),m1_temp(i2)) = 1;
            end
            
%             if connTest(conn_m) == 1
%                 break
%             end
            
            %new conn Test.
            sumConn = sum(conn_m,2);
            if(isempty(find(sumConn==0,1)))
                break
            end
            
            %else
            disp('retrying...')
            
        end
    end
end
