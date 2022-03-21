clear all
for i = 1:8
    evalin('base',['load(''result',num2str(i),''')']);
    idx_z = find(m1==0,1,'first');
    if(~isempty(idx_z))
    m1 = m1(1:idx_z-1);
    m2 = m2(1:idx_z-1);
    IL = IL(1:idx_z-1);
    end
    
    conn_matrix_str.p_10.m1{i} = m1;
    conn_matrix_str.p_10.m2{i} = m2;
    conn_matrix_str.p_10.IL{i} = IL;
    
end

save my300_p1 conn_matrix_str