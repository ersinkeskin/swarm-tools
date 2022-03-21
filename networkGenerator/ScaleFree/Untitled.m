cm = full(connM);
for i = 1:(length(al1)-1)
    for j = (i+1):length(al1)
        if( cm(al1(i),al1(j))==0 && cm(al1(j),al1(i))==0)
            bp =1;
        end
    end
end