sideLength = 40;
nAgents = 3*sideLength^2-3*sideLength+1; 

posx = zeros(nAgents,1);
posy = zeros(nAgents,1);

count = 1;
for i = 1:(2*sideLength-1) %row
    for j = 1:( (2*sideLength-1)-abs(i-sideLength) ) %col
        posx(count) = j - mod(( (2*sideLength-1)-abs(i-sideLength) ),sideLength)*1/2;
        posy(count) = i*sqrt(3)/2;
        count = count+1;
    end
end

count = 1;
clear m1 m2 IL
for i = 1:nAgents-1
    for j = i+1:nAgents
        dist = sqrt( (posx(i)-posx(j))^2 + (posy(i)-posy(j))^2 );
        if dist <= 1.1
            m1(count) = i;
            m2(count) = j;
            IL(count) = dist;
            count = count+1;
        end
    end
end