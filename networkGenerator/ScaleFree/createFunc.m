function [m1,m2,IL,success] = createFunc(conn,numConn,mSize)
success = 1;
idx_nz = find(numConn~=0);
conn = conn(idx_nz);
numConn = numConn(idx_nz);
% load slope2
% mSize = 100;
N = mSize^2;
L = sum(conn.*numConn);

rng('shuffle')
%% conn numConn TO 2xN agentLinks

len = size(conn,2);
agentLinks = [1:N;zeros(1,N)];

c = conn;
nC = numConn;

% for i = 1:N
for i = randperm(N)
    idx = randi(len);
    agentLinks(2,i) = c(idx);
    nC(idx) = nC(idx)-1;
    
    if(nC(idx) == 0)
        nC(idx) = [];
        c(idx) = [];
        len = len-1;
    end
end

%%
connM = spalloc(N,N,L);
len = size(agentLinks,2);

m1 = zeros(L/2,1);
m2 = zeros(L/2,1);
IL = zeros(L/2,1);
lCount = 1;

al1 = agentLinks(1,:);
al2 = agentLinks(2,:);
tic
t = 0;
l=0;
stall_old = -1;
stallCount=0;
while 1
    
    sumTickets = sum(al2);
    if(sumTickets==0)
        break
    end
    
    %%%%
    %     if(sumTickets<10)
    if(sumTickets == stall_old)
        stallCount= stallCount +1;
    else
        stall_old = sumTickets;
        stallCount = 0;
    end
    
    if(stallCount>1000)
        
        if(sumTickets<1000)
            success = 1;
        else
            success = 0;
        end
        
        break
    end
    if(stallCount>500)
        idx_nz = find(al2);
        al2 = al2(idx_nz);
        al1 = al1(idx_nz);
    end
    
    %     end
    
    %%%%
    %     if(sumTickets>10)
    %         if(sumTickets == stall_old)
    %             stallCount= stallCount +1;
    %         else
    %             stall_old = sumTickets;
    %             stallCount = 0;
    %         end
    %
    %         if(stallCount>100)
    %             idx_nz = find(al2);
    %             al2 = al2(idx_nz);
    %             al1 = al1(idx_nz);
    %         end
    %
    %         if(stallCount>1000)
    %
    %             % if fails, rewire!
    %             success = 0;
    %
    %             break
    %         end
    %
    %     end
    
    %%%%%
    ticket = randi(sumTickets);
    count = 0;
    for i = 1:N
        count = count + al2(i);
        if(count>= ticket)
            i1 = i;
            idx = al1(i);
            break
        end
    end
    
    %pull another that is not equal to 1st one
    len_al2 = length(al2);
    while 1
        
        i2 = randi(len_al2);
        idx2 = al1(i2);
        if(idx ~= idx2 && al2(i2)>0)
            break
        end
    end
    
    % if already connected, cont
    if(connM(idx,idx2) == 1 || connM(idx2,idx) == 1)
        continue
        
        
    else
        %else, connect.
        connM(idx,idx2) = 1;
                connM(idx2,idx) = 1;
        
        %record end products
        %given linear indices, find dist between agents in a mSize square matrix
        row1 = floor((idx-1)/mSize)+1;
        col1 = mod(idx-1,mSize)+1;
        
        row2 = floor((idx2-1)/mSize)+1;
        col2 = mod(idx2-1,mSize)+1;
        
        IL(lCount) = sqrt((row1-row2)^2+(col1-col2)^2);
        m1(lCount) = idx;
        m2(lCount) = idx2;
        
        lCount = lCount+1;
        %decrease availabilities
        al2(i1) = al2(i1) -1;
        al2(i2) = al2(i2) -1;
        
        %         disp(lCount)
        if(mod(lCount,1000) ==0)
            
            idx_nz = find(al2);
            al2 = al2(idx_nz);
            al1 = al1(idx_nz);
            
        end
        
        
    end
    
    
end

%%
if(success==1)
    
    twoLinkers = find(agentLinks(2,:)==2);
    
    %look for the id in only m1. if it's a twoLinker, the other one will appear
    %in m1 anyways.
    
    m1two = zeros(size(m1));
    m2two = zeros(size(m2));
    
    for ii = 1:length(twoLinkers)
        
        idx_m1 = find(m1==twoLinkers(ii));
        m1two(idx_m1) = 1;
        
        idx_m2 = find(m2==twoLinkers(ii));
        m2two(idx_m2) = 1;
        
    end
    
    idTwo = find(m1two==1 & m2two==1);
    idTwoShf = idTwo(randperm(length(idTwo)));
    
    %%%%
    
    idx_nz = find(al2);
    al2 = al2(idx_nz);
    al1 = al1(idx_nz);
    
    idxLink = 1;
    while sumTickets>0
        
        idx = m1(idTwoShf(idxLink));
        idx2 = m2(idTwoShf(idxLink));
        connM(idx,idx2) = 0;
        connM(idx2,idx) = 0;
        %link1
        
        while 1
            id_al = randi(length(al1)); %pull one
            idxn = al1(id_al);
            if(connM(idx,idxn) == 0 && connM(idxn,idx) == 0) %if unconn
                connM(idx,idxn) = 1;
                connM(idxn,idx) = 1; %conn
                
                m2(idTwoShf(idxLink)) = idxn; %change m2 id
                
                                row1 = floor((idxn-1)/mSize)+1;
        col1 = mod(idxn-1,mSize)+1;
        
        row2 = floor((idx-1)/mSize)+1;
        col2 = mod(idx-1,mSize)+1;
        
        IL(idTwoShf(idxLink)) = sqrt((row1-row2)^2+(col1-col2)^2);
        
                al2(id_al) = al2(id_al) -1; %dec rease availb
                break
            end
        end
        idx_nz = find(al2); %remove zero entries
        al2 = al2(idx_nz);  %will pull from al1 w/o looking at avlb!
        al1 = al1(idx_nz);
        
        %link2
        while 1
            id_al = randi(length(al1)); %pull one
            idxn = al1(id_al);
            if(connM(idx2,idxn) == 0 && connM(idxn,idx2) == 0)
                connM(idx2,idxn) = 1;
                connM(idxn,idx2) = 1;
                
                m1(lCount) = idx2;
                m2(lCount) = idxn;
                
                row1 = floor((idxn-1)/mSize)+1;
        col1 = mod(idxn-1,mSize)+1;
        
        row2 = floor((idx2-1)/mSize)+1;
        col2 = mod(idx2-1,mSize)+1;
        
        IL(lCount) = sqrt((row1-row2)^2+(col1-col2)^2);
        
                al2(id_al) = al2(id_al) -1; %decrease availb
                lCount = lCount+1;
                break
            end
        end
        idx_nz = find(al2); %remove zero entries
        al2 = al2(idx_nz);  %will pull from al1 w/o looking at avlb!
        al1 = al1(idx_nz);
        
        %
        sumTickets = sum(al2);
        idxLink = idxLink+1;
    end
end

ct = connTest(full(connM));

if(ct==0)
    success = 0;
end