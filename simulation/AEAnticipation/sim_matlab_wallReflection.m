%% posn & ort
% rng(1)
iterLim = 5e4;

ts = 0.1;
vel = 0.002;
alpha = 0.004;
beta = 0.12;
K = -5;
b = -150*0;

angle_rd_ts = 0* 20 /2*pi/180*ts;

widthCoef = 0.8;
obsType = 2;

p = 0.01*0;

sideLength = 8;
nAgents = 3*sideLength^2-3*sideLength+1;

posx = zeros(nAgents,1);
posy = zeros(nAgents,1);
ort = ones(nAgents,1)*0.0;

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
pos2link = zeros(nAgents*6*2,nAgents);
natDist = zeros(nAgents*6*2,1);
for i = 1:nAgents-1
    for j = i+1:nAgents
        dist = sqrt( (posx(i)-posx(j))^2 + (posy(i)-posy(j))^2 );
        if dist <= 1.1
            pos2link(count,i) = 1;
            pos2link(count,j) = -1;
            natDist(count) = sqrt( (posx(i)-posx(j))^2 + (posy(i)-posy(j))^2 );
            count = count+1;
        end
    end
end
nL = count-1;

% link remove
% pos2link = pos2link(1:count-1,:);
% for i = 1:count*p
% idx = randi(size(pos2link,1)); % remove links.
% pos2link(idx,:) = [];
% end
% count = size(pos2link,1)+1;

%add erdös renyi
for i = 1:nL*p
    while 1
        i1 = randi(nAgents);
        i2 = randi(nAgents);
        if i1~=i2
            pos2link(count,i1) = -1;
            pos2link(count,i2) = 1;
            natDist(count) = sqrt( (posx(i1)-posx(i2))^2 + (posy(i1)-posy(i2))^2 );
            count = count+1;
            break
        end
    end
end

nLinks = count-1;

natDist = natDist(1:nLinks,:);
pos2link = pos2link(1:nLinks,:);
pos2link = sparse(pos2link);
link2pos = pos2link';
% distLink1 = ones(nLinks,1);
distLink1 = natDist;
velAgent = ones(nAgents,1)*vel;

posx_log = zeros(iterLim,nAgents);
posy_log = zeros(iterLim,nAgents);
velx_log = zeros(iterLim,nAgents);
vely_log = zeros(iterLim,nAgents);
ort_log = zeros(iterLim,nAgents);
psi = zeros(iterLim,1);
%%
beta_ts = beta*ts;
bTs = b/ts;

%% obstacle
obsX = 15;
obsX = 10;
% obsY = 8.85;
obsR = 15;
obsY = (sideLength-1)*sqrt(3)/2*widthCoef + obsR;


obsD = obsY-obsR;
obsK = 0.1*1;
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
    
    %     F = ( bTs*(distLink-distLink1) + K*(distLink-1) ) ./distLink;
%     F = ( bTs*(distLink-distLink1) + K*(distLink-natDist) ) ./distLink./natDist;
    F = ( bTs*(distLink-distLink1) + K*(distLink-natDist) ) ./distLink;
    distLink1 = distLink;
    agentFx = link2pos*(F.*xLink);
    agentFy = link2pos*(F.*yLink);
    
    
    %obstacle
    switch obsType
        case 1 % passage
            for j = 1:size(obsList,1)
                xObs = posx-obsList(j,1);
                yObs = posy-obsList(j,2);
                rObs = sqrt( xObs.^2 + yObs.^2 );
                fObs = obsK./(rObs-obsList(j,3)).^2.*(rObs<(obsList(j,3)+obsD));
                
                agentFx = agentFx + fObs.*xObs./rObs;
                agentFy = agentFy + fObs.*yObs./rObs;
            end
        case 2 %duvar
            xObs = posx-obsList(1,1);
%             yObs = posy-obsList(j,2);
%             rObs = sqrt( xObs.^2 + yObs.^2 );
            rObs = xObs;
%             fObs = obsK./(rObs-obsList(j,3)).^2.*(rObs<(obsList(j,3)+obsD));
            fObs = -obsK./(rObs).^2.*(rObs<obsD);
            
            agentFx = agentFx + fObs;
%             agentFy = agentFy + fObs.*yObs./rObs;
            
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
    ort = ort + beta_ts*(-agentFx.*sin_ort+agentFy.*cos_ort) + (rand([nAgents,1])*2-1)*angle_rd_ts;
    %     ort(end) = pi/4;
    posx_log(i,:) = posx;
    posy_log(i,:) = posy;
    velx_log(i,:) = velx;
    vely_log(i,:) = vely;
    ort_log(i,:) = ort;
    psi(i) = sqrt(sum(cos_ort)^2+sum(sin_ort)^2)/nAgents;
end

%plotSim2;
