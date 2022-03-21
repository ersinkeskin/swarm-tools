% c = gcp('nocreate');    % start matlab parallel pool
% delete(c);
% p=parcluster('local');
% p.NumWorkers=4;
% parpool(4);

numDiags = 16;
mSize = 100;

m1 = cell(numDiags,1);
m2 = cell(numDiags,1);
IL = cell(numDiags,1);

load olmusslope30_Kmax1
ii=1

%%
parfor_progress(numDiags);
for ii=ii:numDiags

    [m1{ii},m2{ii},IL{ii}]=createFunc(bp,N2,mSize);
    parfor_progress();
end
parfor_progress(0);
%%
conn_matrix_str.m1 = m1;
conn_matrix_str.m2 = m2;
conn_matrix_str.IL = IL;