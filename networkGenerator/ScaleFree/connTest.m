function connBit = connTest(conn_m)

stNode = 1;
visited = zeros(1,size(conn_m,1));
% parentNode = zeros(1,size(conn_m,1));
visited(stNode) = 1;
queue = stNode;

sizeConn = size(conn_m,1);

while 1
    [queue, idx_pop] = queue_pop(queue);
    
    for idx_n = 1:sizeConn
        if(conn_m(idx_n,idx_pop)~=0 && visited(idx_n)==0)
            
            queue = [queue idx_n];
            %             parentNode(idx_n) = idx_pop;
            visited(idx_n) = 1;
        end
    end
    
    if(isempty(queue))
        break
    end
end

if(length(find(visited))==sizeConn)
    connBit = 1;
else
    connBit = 0;
end

end


function [queue_return, element] = queue_pop(queue)

if(isempty(queue))
    element = [];
    queue_return=[];
    return
else if(length(queue)==1)
        element = queue(1);
        queue_return = [];
        return
    else
        element = queue(1);
        queue_return = queue(2:end);
        return
    end
end
end