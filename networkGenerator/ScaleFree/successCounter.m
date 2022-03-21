numDiags = 16;
mSize = 100;

m1 = cell(numDiags,1);
m2 = cell(numDiags,1);
IL = cell(numDiags,1);

load olmusslope40_Kmax1
ii=1
counter = 0;
%%
parfor_progress(numDiags);
while ii<=numDiags

    [m1{ii},m2{ii},IL{ii},success]=createFunc(bp,N2,mSize);
    if(success==1)
        ii = ii+1;
        parfor_progress();
    end
    
    counter = counter +1;
    disp(ii)
    disp(counter)
end
parfor_progress(0);
disp(round(numDiags/counter*100))
%%
conn_matrix_str.m1 = m1;
conn_matrix_str.m2 = m2;
conn_matrix_str.IL = IL;