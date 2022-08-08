 global ts alpha vel beta Kacc n

%% SIM KISMI posn & ort
iterLim = 5e2;
tic

ts = 0.05;
vel = 1;
alpha = 0.5;
beta = 0.12;
Ksp = 5;
Kacc = 0.1;
b = 25*1;

sideLength = 3;
nAgents = 3*sideLength^2-3*sideLength+1;

posx = zeros(nAgents,1);
posy = zeros(nAgents,1);
ort = ones(nAgents,1)*-pi/12*0;

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
countCenter = 2;
centerAgent = (nAgents+1)/2;
centerIds = [ centerAgent, zeros(1,6)];
pos2link = zeros(nAgents*6,nAgents);
for i = 1:nAgents-1
    for j = i+1:nAgents
        dist = sqrt( (posx(i)-posx(j))^2 + (posy(i)-posy(j))^2 );
        if dist <= 1.1
            pos2link(count,i) = 1;
            pos2link(count,j) = -1;
            count = count+1;
            
            if(i==centerAgent)
                centerIds( countCenter ) = j;
                countCenter = countCenter+1;
            elseif(j==centerAgent)
                centerIds( countCenter ) = i;
                countCenter = countCenter+1;
            end
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

beta_ts = beta*ts;
bTs = b/ts;

% obstacle
obsX = 15;
% obsY = 8.85;
obsY = 1.00;
obsR = 3;

obsD = 10;%obsY-obsR;
obsK = 30;
obsList = [obsX obsY obsR;
    obsX -obsY-10 obsR];
% obsList = [obsX obsY obsR];
% obsList = [obsX*9 0 100];

%% KF KISMI
syms tau real

n = 4;
nr = 7;
N = n*nr; %states [x,y,v,psi,psidot]

%meas
H1 = zeros(3,n);
H1(1,1) = 1;
H1(2,2) = 1;
H1(3,4) = 1;

H2 = zeros(2,n);
H2(1,1) = 1;
H2(2,2) = 1;

H = blkdiag(H1,H2,H2,H2,H2,H2,H2);
for i = 4:2:14
    H(i,1) = -1;
    H(i+1,2) = -1;
end

sigmaPos    = 0.03;
sigmaOrt   = 5 *pi/180; %measurement std.
Rk          = diag(ones(size(H,1),1)*sigmaPos^2);
Rk(3,3) = sigmaOrt^2;

xpre = zeros(N,1); %just to make col vect

xr = zeros(N,1);
xk = zeros(N,1);
ortMeas = ort(centerIds(1)) + sigmaOrt*randn;
for i = 1:nr
    xk(1 + (i-1)*n) = posx(centerIds(i)) + sigmaPos*randn;
    xk(2 + (i-1)*n) = posy(centerIds(i)) + sigmaPos*randn;
    xk(3 + (i-1)*n) = vel;
    xk(4 + (i-1)*n) = ortMeas;
end


% P_k = diag([0.1*ones(N,1)]); %init covariance
% P_k = diag( repmat( [sigmaPos^2 sigmaPos^2 vel^2 pi^2]',[nr,1]));
P_k = diag( repmat( [sigmaPos^2 sigmaPos^2 (0.05)^2 (sigmaOrt)^2]',[nr,1]));
q3 = 0.001;
q4 = 0.01;
q3o = q3*20;
q4o = q4*20;

% q4 = 0.001;
% Q = diag([0 0 q3 0 q5]);

%allocate
xkArr   = zeros(N,iterLim);
xpreArr   = zeros(N,iterLim);
xrArr   = zeros(N,iterLim);
measArr = zeros(size(H,1),iterLim);
Parr    = zeros(N,iterLim);
Karr    = zeros(size(H,1)*size(H,2),iterLim);

fs = cell(nr,1);
Qks = cell(nr,1);
xks = cell(nr,1);
xpres = cell(nr,1);

xkdiv = reshape(xk,[n,nr]);
for ir = 1:nr
    xks{ir} = xkdiv(:,ir);
end

%%
% ort(end) = pi/4;
for i = 1:iterLim
    
    xLink = pos2link*posx;
    yLink = pos2link*posy;
    distLink = sqrt(xLink.^2+yLink.^2);
    
    Force = ( -bTs*(distLink-distLink1) + -Ksp*(distLink-1) ) ./distLink;
    distLink1 = distLink;
    agentFx = link2pos*(Force.*xLink);
    agentFy = link2pos*(Force.*yLink);
    
    
    agentFxObs = 0;
    agentFyObs = 0;
    %     obstacle
    for j = 1:size(obsList,1)
        xObs = posx-obsList(j,1);
        yObs = posy-obsList(j,2);
        rObs = sqrt( xObs.^2 + yObs.^2 );
        fObs = obsK./(rObs-obsList(j,3)).^2.*(rObs<(obsList(j,3)+obsD));
        
        agentFxObs = agentFxObs + fObs.*xObs./rObs;
        agentFyObs = agentFyObs + fObs.*yObs./rObs;
    end
    
%         xObs = posx-obsList(1,1);
%     agentFxObs = -obsK./(xObs).^2.*(xObs<obsD);
    
    
    agentFx = agentFx + agentFxObs;
    agentFy = agentFy + agentFyObs;
    
    % itgt
    cos_ort = cos(ort);
    sin_ort = sin(ort);
    
    fparallel_obs = (agentFxObs.*cos_ort+agentFyObs.*sin_ort);
    fperp_obs = (-agentFxObs.*sin_ort+agentFyObs.*cos_ort);
    
    fparallel_obs = fparallel_obs(centerIds(1));
    fperp_obs = fperp_obs(centerIds(1));
    
    vSetp = (agentFx.*cos_ort+agentFy.*sin_ort)*alpha + vel;
    velAgent = velAgent + Kacc*(vSetp-velAgent)*ts;
    velx = velAgent.*cos_ort;
    vely = velAgent.*sin_ort;
    posx = posx + velx*ts;
    posy = posy + vely*ts;
    ortdot = beta*(-agentFx.*sin_ort+agentFy.*cos_ort);
    ort = ort + ortdot*ts;
    
    fparallel_model = (agentFx.*cos_ort+agentFy.*sin_ort);
    fperp_model = (-agentFx.*sin_ort+agentFy.*cos_ort);
        fparallel_model = fparallel_model(centerIds(1));
    fperp_model = fperp_model(centerIds(1));
    %     posx = posx + velx*ts;
    %     posy = posy + vely*ts;
    %     vSetp = (agentFx.*cos_ort+agentFy.*sin_ort)*alpha + vel;
    %     velAgent = velAgent + (vSetp-velAgent)*ts;
    %     velx = velAgent.*cos_ort;
    %     vely = velAgent.*sin_ort;
    %
    %     ort = ort + ortdot*ts;
    %     ortdot = beta*(-agentFx.*sin_ort+agentFy.*cos_ort);
    
    
    % KALMAN BURAYA.
    %unicycle7 modelini buraya yedirebilmeliyiz. ustu sim iste.
    
    %sim
    for ir = 1:nr
        xr(1 + (ir-1)*n) = posx(centerIds(ir));
        xr(2 + (ir-1)*n) = posy(centerIds(ir));
        xr(3 + (ir-1)*n) = velAgent(centerIds(ir));
        xr(4 + (ir-1)*n) = ort(centerIds(ir));
        %         xr(5 + (ir-1)*n) = ortdot(centerIds(ir));
    end
    
    %% F,phiK,Qk
    
    for ir = 1:nr
        fs{ir} = ffunc(xks{ir});
    end
    F = blkdiag(fs{1},fs{2},fs{3},fs{4},fs{5},fs{6},fs{7}); %buras? da otomatize edilmeli.
    
    x1 = xk(1);
    y1 = xk(2);
    v1 = xk(3);
    th1 = xk(4);
    
    u1 = [cos(th1);sin(th1)];
    u1p = [-sin(th1);cos(th1)];
    
    fparallel_x1 = 0;
    fparallel_y1 = 0;
    fparallel_v1 = 0;
    fparallel_th1 = 0;
    
    fperp_x1 = 0;
    fperp_y1 = 0;
    fperp_v1 = 0;
    fperp_th1 = 0;
    
    fparallel = 0; %for state propgt
    fperp = 0;
    
    for ir = 2:nr
        
        xf = xk(1 + (ir-1)*n);
        yf = xk(2 + (ir-1)*n); %xfocus,yfocus gibi
        vf = xk(3 + (ir-1)*n);
        thf = xk(4 + (ir-1)*n);
        df = sqrt( (xf - xk(1))^2 + (yf - xk(2))^2 );
        
        ulink = [xf-x1; yf-y1]/df;
        un = [cos(thf); sin(thf)];
        unp = [-sin(thf); cos(thf)];
        
        u1ulink = dot(u1,ulink);
        u1pulink = dot(u1p,ulink);
        unulink = dot(un,ulink);
        unpulink = dot(unp,ulink);
        
        fparallel = fparallel + Ksp*(df-1)*u1ulink + b*(vf*unulink-v1*u1ulink)*u1ulink;
        fperp = fperp + Ksp*(df-1)*u1pulink + b*(vf*unulink-v1*u1ulink)*u1pulink;
        
        %***vdot derivatives
        
        %xn
        fspring_xn = Ksp*cos(th1)*(1-1/df);
        fdamper_xn = b/df*( u1ulink*(vf*cos(thf)-v1*cos(th1)) + cos(th1)*(vf*unulink-v1*u1ulink));
        
        fparallel_x1 = fparallel_x1 - fspring_xn - fdamper_xn;
        fparallel_xn = fspring_xn + fdamper_xn;
        
        %yn
        fspring_yn = Ksp*sin(th1)*(1-1/df);
        fdamper_yn = b/df*( u1ulink*(vf*sin(thf)-v1*sin(th1)) + sin(th1)*(vf*unulink-v1*u1ulink));
        
        fparallel_y1 = fparallel_y1 - fspring_yn - fdamper_yn;
        fparallel_yn = fspring_yn + fdamper_yn;
        
        %vn
        fparallel_vn = b*u1ulink*unulink;
        fparallel_v1 = fparallel_v1 -b*u1ulink^2;
        
        %thetan
        fparallel_thn = b*vf*u1ulink*unpulink;
        fparallel_th1 = fparallel_th1 + Ksp*(1-1/df)*u1pulink + b*vf*unulink*u1pulink - b*2*v1*u1pulink*u1ulink;
        
        
        %***thetadot derivatives
        %xn
        fspring_xn = -Ksp*sin(th1)*(1-1/df);
        fdamper_xn = b/df*( u1pulink*(vf*cos(thf)-v1*cos(th1)) - sin(th1)*(vf*unulink-v1*u1ulink));
        
        fperp_x1 = fperp_x1 - fspring_xn - fdamper_xn;
        fperp_xn = fspring_xn + fdamper_xn;
        
        %yn
        fspring_yn = Ksp*cos(th1)*(1-1/df);
        fdamper_yn = b/df*( u1pulink*(vf*sin(thf)-v1*sin(th1)) + cos(th1)*(vf*unulink-v1*u1ulink));
        
        fperp_y1 = fperp_y1 - fspring_yn -  fdamper_yn;
        fperp_yn = fspring_yn + fdamper_yn;
        
        %vn
        fperp_vn = b*u1pulink*unulink;
        fperp_v1 = fperp_v1 -b*u1pulink*u1ulink;
        
        %theta n
        fperp_thn = b*vf*u1pulink*unpulink;
        fperp_th1 = fperp_th1 - Ksp*(1-1/df)*u1ulink - b*vf*unulink*u1ulink + b*v1*( u1ulink^2 - u1pulink^2 );
        
        %%%% ***F matrisine atama
        %vdot
        F(3, 1 + (ir-1)*n) = Kacc*alpha*fparallel_xn;
        F(3, 2 + (ir-1)*n) = Kacc*alpha*fparallel_yn;
        F(3, 3 + (ir-1)*n) = Kacc*alpha*fparallel_vn;
        F(3, 4 + (ir-1)*n) = Kacc*alpha*fparallel_thn;
        %thdot
        F(4, 1 + (ir-1)*n) = beta*fperp_xn;
        F(4, 2 + (ir-1)*n) = beta*fperp_yn;
        F(4, 3 + (ir-1)*n) = beta*fperp_vn;
        F(4, 4 + (ir-1)*n) = beta*fperp_thn;
        
    end
    
    %en son 1e gore turevler
    F(3,1) = Kacc*alpha*fparallel_x1;
    F(3,2) = Kacc*alpha*fparallel_y1;
    F(3,3) = Kacc*alpha*fparallel_v1 - Kacc;
    F(3,4) = Kacc*alpha*fparallel_th1;
    
    F(4, 1 ) = beta*fperp_x1;
    F(4, 2 ) = beta*fperp_y1;
    F(4, 3 ) = beta*fperp_v1;
    F(4, 4 ) = beta*fperp_th1;
    
    fparallel = fparallel + fparallel_obs*1;
    fperp = fperp + fperp_obs*1;
    
    %direk modelden
%     fparallel = fparallel_model;
%     fperp = fperp_model;
    

    
    %% **** Q FUNC ****
    Q_kalman4
    
    %%
    phiK = eye(N) + F*ts;
    
    %     for ir = 1:nr
    %         if(ir==1)
    %             Qks{ir} = Qkfunc_firstAgent(xks{ir});
    %         else
    %             Qks{ir} = Qkfunc(xks{ir});
    %         end
    %     end
    %     Qk = blkdiag(Qks{1},Qks{2},Qks{3},Qks{4},Qks{5},Qks{6},Qks{7}); %buras? da otomatize edilmeli.
    
    %     phiTau = eye(N) + F*tau;
    %     Qk = double(int(phiTau*Q*phiTau',tau,0,ts));
    
    %%
    
    %3. model apriori
    for ir = 1:nr
        if(ir==1)
            xpres{ir} = statePropagate_firstAgent(xks{ir},fparallel,fperp);
        else
            xpres{ir} = statePropagate(xks{ir});
        end
        xpre( (1:n) + n*(ir-1) ) = xpres{ir};
    end
    %     xpre = [xpres{1};xpres{2}];
    
    %%
    %4. covariance propogation
    Mk = phiK*P_k*phiK' + Qk;
    
    %5. calculate Kalman gain
    K = Mk*H'*(H*Mk*H' + Rk)^-1;
    
    %6. get measurement
    meas = H*xr + [1*sigmaPos*randn(2,1); sigmaOrt*randn(1,1); sigmaPos*randn(12,1)]; %buraya otomatizxasyon
    
    %7. create new estimation
    xk = xpre + K*(meas - H*xpre);
    xkdiv = reshape(xk,[n,nr]);
    for ir = 1:nr
        xks{ir} = xkdiv(:,ir);
    end
    
    %8. update covariance
    P_k = (eye(N)-K*H)*Mk;
    
    %LOG
    xkArr(:,i)      = [xk];
    xpreArr(:,i)      = [xpre];
    xrArr(:,i) = xr;
    measArr(:,i) = meas;
    Parr (:,i)      = diag(P_k);
    Karr(:,i)       = reshape(K,[size(H,1)*size(H,2) 1]);
    
    %% KALMAN END
    
    %% LOG
    %     ort(end) = pi/4;
    posx_log(i,:) = posx;
    posy_log(i,:) = posy;
    velx_log(i,:) = velx;
    vely_log(i,:) = vely;
    ort_log(i,:) = ort;
end

toc

% plotSim2;


%% functions

function F = ffunc(xk)
f13 = cos(xk(4));
f14 = -xk(3)*sin(xk(4));
f23 = sin(xk(4));
f24 = xk(3)*cos(xk(4));

F = [0 0 f13 f14;
    0 0 f23 f24 ;
    0 0 0 0 ;
    0 0 0 0 ];
end


function xkp = statePropagate(xk)
global ts n
xkp = zeros(n,1);
xkp(1) = xk(1) + xk(3)*cos(xk(4))*ts;
xkp(2) = xk(2) + xk(3)*sin(xk(4))*ts;
xkp(3) = xk(3);%1 + 0.1*sin(2*pi*0.1* i*ts);
xkp(4) = xk(4);
% xkp(5) = xk(5);%xr1(5) + 0.1;
end


function xkp = statePropagate_firstAgent(xk,fparallel,fperp)
global ts alpha vel beta Kacc n
xkp = zeros(n,1);
xkp(1) = xk(1) + xk(3)*cos(xk(4))*ts;
xkp(2) = xk(2) + xk(3)*sin(xk(4))*ts;
xkp(3) = xk(3) + ts*Kacc*(vel + fparallel*alpha - xk(3));%1 + 0.1*sin(2*pi*0.1* i*ts);
xkp(4) = xk(4) + beta*fperp*ts;
end