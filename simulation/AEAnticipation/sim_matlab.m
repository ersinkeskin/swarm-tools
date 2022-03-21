%% posn & ort
iterLim = 1.3e5;

ts = 0.1;
vel = 0.002;
alpha = 0.004;
beta = 0.12;
K = -5;
b = -150*1;

sideLength = 10;
nAgents = 3*sideLength^2-3*sideLength+1;

posx = zeros(nAgents,1);
posy = zeros(nAgents,1);
ort = zeros(nAgents,1);

count = 1;
for i = 1:(2*sideLength-1) %row
    for j = 1:( (2*sideLength-1)-abs(i-sideLength) ) %col
        posx(count) = j - mod(( (2*sideLength-1)-abs(i-sideLength) ),sideLength)*1/2;
        posy(count) = i*sqrt(3)/2;
        count = count+1;
    end
end
posx = posx-mean(posx);
posy = posy-mean(posy);

count = 1;
pos2link = zeros(nAgents*6,nAgents);
for i = 1:nAgents-1
    for j = i+1:nAgents
        dist = sqrt( (posx(i)-posx(j))^2 + (posy(i)-posy(j))^2 );
        if dist <= 1.1
            pos2link(count,i) = 1;
            pos2link(count,j) = -1;
            count = count+1;
        end
    end
end
nLinks = count-1;
pos2link = pos2link(1:nLinks,:);
pos2link = sparse(pos2link);
link2pos = pos2link';
distLink1 = ones(nLinks,1);
velAgent = ones(nAgents,1)*vel;

posx_log = zeros(iterLim,nAgents);
posy_log = zeros(iterLim,nAgents);
velx_log = zeros(iterLim,nAgents);
vely_log = zeros(iterLim,nAgents);
ort_log = zeros(iterLim,nAgents);
%%
beta_ts = beta*ts;
bTs = b/ts;

%% obstacle
obsX = 15;
% obsY = 8.85;
obsY = 9.00;
obsR = 3;

obsD = 6;%obsY-obsR;
obsK = 1.4;
obsList = [obsX obsY obsR;
    obsX -obsY obsR];
% obsList = [obsX obsY obsR];
% obsList = [obsX*9 0 100];
%%
% ort(end) = pi/4;
for i = 1:iterLim
    
    xLink = pos2link*posx;
    yLink = pos2link*posy;
    distLink = sqrt(xLink.^2+yLink.^2);
    
    F = ( bTs*(distLink-distLink1) + K*(distLink-1) ) ./distLink;
    distLink1 = distLink;
    agentFx = link2pos*(F.*xLink);
    agentFy = link2pos*(F.*yLink);
    
    
    %obstacle
    for j = 1:size(obsList,1)
        xObs = posx-obsList(j,1);
        yObs = posy-obsList(j,2);
        rObs = sqrt( xObs.^2 + yObs.^2 );
        fObs = obsK./(rObs-obsList(j,3)).^2.*(rObs<(obsList(j,3)+obsD));
        
        agentFx = agentFx + fObs.*xObs./rObs;
        agentFy = agentFy + fObs.*yObs./rObs;
    end
    
    % itgt
    cos_ort = cos(ort);
    sin_ort = sin(ort);
    
    vSetp = (agentFx.*cos_ort+agentFy.*sin_ort)*alpha + vel;
    velAgent = velAgent + (vSetp-velAgent)*ts;
    velx = velAgent.*cos_ort;
    vely = velAgent.*sin_ort;
    posx = posx + velx*ts;
    posy = posy + vely*ts;
    ort = ort + beta_ts*(-agentFx.*sin_ort+agentFy.*cos_ort);
%     ort(end) = pi/4;
    posx_log(i,:) = posx;
    posy_log(i,:) = posy;
    velx_log(i,:) = velx;
    vely_log(i,:) = vely;
    ort_log(i,:) = ort;
end

plotSim2;