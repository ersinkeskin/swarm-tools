mSize = 16;
connTh = 1.42;

%%
nAgents = mSize^2;

%%

count = 1;
m1 = zeros(nAgents*8,1);
m2 = zeros(nAgents*8,1);
IL = zeros(nAgents*8,1);

for i = 1:(nAgents-1)
    for j = (i+1):nAgents
%             [row1,col1] = ind2rowcol(i,mSize);
    row1 = floor((i-1)/mSize)+1;
    col1 = mod(i-1,mSize)+1;
%     [row2,col2] = ind2rowcol(j,mSize);
    row2 = floor((j-1)/mSize)+1;
    col2 = mod(j-1,mSize)+1;
    
    dist = sqrt((row1-row2)^2+(col1-col2)^2);
        if(dist<connTh)
            m1(count) = i;
            m2(count) = j;
            IL(count) = dist;
            count = count+1;
        end
    end
   if(mod(i,mSize)==0)
       disp(i)
   end
end

m1 = m1(1:count-1);
m2 = m2(1:count-1);
IL = IL(1:count-1);

save result m1 m2 IL