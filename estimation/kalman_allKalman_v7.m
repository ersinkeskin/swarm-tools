%% __INITIALIZATION__

global ts alpha vel beta Kacc n
%% MODEL
obs_normWidth = 0.8;

addNoise = 1

iterLim = 1000;
tic

sideLength = 3;
nAgents = 3*sideLength^2-3*sideLength+1;
iFocal = ceil(nAgents/2);

dn = 1;

ts = 0.05*1;
vel = 0.3;
alpha = 0.1;
beta = 0.4;
Ksp = vel/(dn*0.3*alpha);
Kacc = 0.1;
b = 10*1;

sigmaPos = 0.03;
sigmaOrt = 5 *pi/180;

q1 = 0.01;
q3 = 0.01;
q4 = 0.1;
q3o = q3*1;%100;
q4o = q4*10;%20000;

ts2 = ts^2;
ts3 = ts^3;

%% degismeyecekler
posx = zeros(nAgents,1);
posy = zeros(nAgents,1);
ort = ones(nAgents,1)*0;
velAgent = ones(nAgents,1)*vel;

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

posx = posx*dn;
posy = posy*dn;

% posy = posy-0.5;
%%%%NN NETWORK
count = 1;
pos2link = zeros(nAgents*6,nAgents);
adjcMatrix =  zeros(nAgents,nAgents);
for i = 1:nAgents-1
    for j = i+1:nAgents
        dist = sqrt( (posx(i)-posx(j))^2 + (posy(i)-posy(j))^2 );
        if dist <= dn*1.01
            pos2link(count,i) = 1;
            pos2link(count,j) = -1;
            count = count+1;
            
            adjcMatrix(i,j) = 1;
        end
    end
end
nLinks = count-1;
pos2link = pos2link(1:nLinks,:);
pos2link = sparse(pos2link);
link2pos = pos2link';
adjcMatrix = adjcMatrix+adjcMatrix';

centerIdsArr = cell(nAgents,1);
nRobotsArr = zeros(nAgents,1);
for i = 1:nAgents
    centerIdsArr{i} = [i find(adjcMatrix(i,:))];
    nRobotsArr(i) = length(find(adjcMatrix(i,:))) + 1;
end

%% KALMAN FILTER
n = 4; %[x,y,v,theta]

nzeros = zeros(n);
Fmaster = zeros(7*n);
H1 = [1,0,0,0; 0,1,0,0; 0,0,0,1];
H2 = [1,0,0,0; 0,1,0,0];
H = blkdiag(H1,H2,H2,H2,H2,H2,H2);
for i = 4:2:14
    H(i,1) = -1;
    H(i+1,2) = -1;
end

H6 = H;
H4 = H(1:(3+4*2),1:(n*5));
H3 = H(1:(3+3*2),1:(n*4));

Rk          = diag(ones(size(H,1),1)*sigmaPos^2);
Rk(3,3) = sigmaOrt^2;

Rk6 = Rk;
Rk4 = Rk(1:size(H4,1),1:size(H4,1));
Rk3 = Rk(1:size(H3,1),1:size(H3,1));

xks = cell(nAgents,1);
Pks = cell(nAgents,1);

%% obstacle
obsD = (sideLength-1)*sqrt(3)*obs_normWidth/2;

obsX = 5;
obsR = 3.0;

obsY = obsR + obsD;

obsK = 0.15*3;
maxFobs = vel/alpha*1.5;

obsList = [obsX obsY obsR;
    obsX -obsY obsR];

%% initialize
for ik = 1:nAgents
    nRobots = nRobotsArr(ik);
    centerIds = centerIdsArr{ik};
    xk = zeros(nRobots*n,1);
    for ir = 1:nRobots
        xk(1 + (ir-1)*n) = posx(centerIds(ir));
        xk(2 + (ir-1)*n) = posy(centerIds(ir));
        xk(3 + (ir-1)*n) = velAgent(centerIds(ir));
        xk(4 + (ir-1)*n) = ort(centerIds(ir));
    end
    xks{ik} = xk;
    
    %     P_k = diag( repmat( [sigmaPos^2 sigmaPos^2 (0.05)^2 (sigmaOrt)^2]',[nRobots,1]));
    P_k = diag( repmat( [sigmaPos^2 sigmaPos^2 (0.05)^2 (0.01)^2]',[nRobots,1]));
    Pks{ik} = P_k;
end

acomArr = zeros(nAgents,1);
ortdotArr = zeros(nAgents,1);

posx_log = zeros(iterLim,nAgents);
posy_log = zeros(iterLim,nAgents);
velx_log = zeros(iterLim,nAgents);
vely_log = zeros(iterLim,nAgents);
ort_log = zeros(iterLim,nAgents);

xkArr   = zeros(nRobotsArr(iFocal)*n,iterLim);
xpreArr   = zeros(nRobotsArr(iFocal)*n,iterLim);
xrArr   = zeros(nRobotsArr(iFocal)*n,iterLim);
measArr = zeros(nRobotsArr(iFocal)*2+1,iterLim);
Parr    = zeros(nRobotsArr(iFocal)*n,iterLim);

ftemp = cell(7,1);
qtemp = cell(7,1);
%% __SIM__
tic
for i = 1:iterLim
    %log
    cos_ort = cos(ort);
    sin_ort = sin(ort);
    velx = velAgent.*cos_ort;
    vely = velAgent.*sin_ort;
    
    posx_log(i,:) = posx;
    posy_log(i,:) = posy;
    velx_log(i,:) = velx;
    vely_log(i,:) = vely;
    ort_log(i,:) = ort;
    
    posx = posx + velx*ts;
    posy = posy + vely*ts;
    velAgent = velAgent + acomArr*ts;
    ort = ort + ortdotArr*ts;
    
    %calculations for robots.
    for ik = 1:nAgents
        
        nRobots = nRobotsArr(ik);
        N = nRobots*n;
        xk = xks{ik};
        P_k = Pks{ik};
        centerIds = centerIdsArr{ik};
        xr = zeros(nRobots*n,1);
        xpre = zeros(nRobots*n,1);
        
        for ir = 1:nRobots
            xr(1 + (ir-1)*n) = posx(centerIds(ir));
            xr(2 + (ir-1)*n) = posy(centerIds(ir));
            xr(3 + (ir-1)*n) = velAgent(centerIds(ir));
            xr(4 + (ir-1)*n) = ort(centerIds(ir));
        end
        
        %get meas
        switch nRobots
            case 4
                H = H3;
                Rk = Rk3;
                %                 meas = H3*xr + [1*sigmaPos*randn(2,1); sigmaOrt*randn(1,1); sigmaPos*randn(6,1)];
                meas = H3*xr + addNoise*[1*sigmaPos*randn(2,1); sigmaOrt*randn(1,1);  sigmaPos*randn(6,1)];
            case 5
                H = H4;
                Rk = Rk4;
                meas = H4*xr + addNoise*[1*sigmaPos*randn(2,1); sigmaOrt*randn(1,1);  sigmaPos*randn(8,1)];
            case 7
                H = H6;
                Rk = Rk6;
                meas = H6*xr + addNoise*[1*sigmaPos*randn(2,1); sigmaOrt*randn(1,1);  sigmaPos*randn(12,1)];
        end
        
        %% FMATRIX
        
        for ir = 1:nRobots
            f1_3 = cos(xk((4)+(ir-1)*n));
            f1_4 = -xk((3)+(ir-1)*n)*sin(xk((4)+(ir-1)*n));
            f2_3 = sin(xk((4)+(ir-1)*n));
            f2_4 = xk((3)+(ir-1)*n)*cos(xk((4)+(ir-1)*n));
            
            ftemp{ir} = [0 0 f1_3 f1_4;
                0 0 f2_3 f2_4 ;
                0 0 0 0 ;
                0 0 0 0 ];
            
            if ir == 1
                q3t = q3;
                q4t = q4;
            else
                q3t = q3o;
                q4t = q4o;
            end
            
            qtemp{ir} = [((q3t*f1_3^2)/3 + (q4t*f1_4^2)/3)*ts3 + q1*ts,       (ts3*(f1_3*f2_3*q3t + f1_4*f2_4*q4t))/3, (f1_3*q3t*ts2)/2, (f1_4*q4t*ts2)/2;
                (ts3*(f1_3*f2_3*q3t + f1_4*f2_4*q4t))/3, ((q3t*f2_3^2)/3 + (q4t*f2_4^2)/3)*ts3 + q1*ts, (f2_3*q3t*ts2)/2, (f2_4*q4t*ts2)/2 ;
                (f1_3*q3t*ts2)/2,                             (f2_3*q3t*ts2)/2,            q3t*ts,                0;
                (f1_4*q4t*ts2)/2,                             (f2_4*q4t*ts2)/2,                0,            q4t*ts ];
            
        end
        
        if nRobots==4
            F = blkdiag(ftemp{1},ftemp{2},ftemp{3},ftemp{4});
            Qk = blkdiag(qtemp{1},qtemp{2},qtemp{3},qtemp{4});
        elseif nRobots==5
            F = blkdiag(ftemp{1},ftemp{2},ftemp{3},ftemp{4},ftemp{5});
            Qk = blkdiag(qtemp{1},qtemp{2},qtemp{3},qtemp{4},qtemp{5});
        elseif nRobots == 6
            F = blkdiag(ftemp{1},ftemp{2},ftemp{3},ftemp{4},ftemp{5},ftemp{6});
            Qk = blkdiag(qtemp{1},qtemp{2},qtemp{3},qtemp{4},qtemp{5},qtemp{6});
        else
            F = blkdiag(ftemp{1},ftemp{2},ftemp{3},ftemp{4},ftemp{5},ftemp{6},ftemp{7});
            Qk = blkdiag(qtemp{1},qtemp{2},qtemp{3},qtemp{4},qtemp{5},qtemp{6},qtemp{7});
        end
        
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
        
        for ir = 2:nRobots
            
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
            
            %***vdot derivatives
            
            %xn
            fspring_xn = Ksp*cos(th1)*(1-dn/df);
            fdamper_xn = b/df*( u1ulink*(vf*cos(thf)-v1*cos(th1)) + cos(th1)*(vf*unulink-v1*u1ulink));
            
            fparallel_x1 = fparallel_x1 - fspring_xn - fdamper_xn;
            fparallel_xn = fspring_xn + fdamper_xn;
            
            %yn
            fspring_yn = Ksp*sin(th1)*(1-dn/df);
            fdamper_yn = b/df*( u1ulink*(vf*sin(thf)-v1*sin(th1)) + sin(th1)*(vf*unulink-v1*u1ulink));
            
            fparallel_y1 = fparallel_y1 - fspring_yn - fdamper_yn;
            fparallel_yn = fspring_yn + fdamper_yn;
            
            %vn
            fparallel_vn = b*u1ulink*unulink;
            fparallel_v1 = fparallel_v1 -b*u1ulink^2;
            
            %thetan
            fparallel_thn = b*vf*u1ulink*unpulink;
            fparallel_th1 = fparallel_th1 + Ksp*(1-dn/df)*u1pulink + b*vf*unulink*u1pulink - b*2*v1*u1pulink*u1ulink;
            
            
            %***thetadot derivatives
            %xn
            fspring_xn = -Ksp*sin(th1)*(1-dn/df);
            fdamper_xn = b/df*( u1pulink*(vf*cos(thf)-v1*cos(th1)) - sin(th1)*(vf*unulink-v1*u1ulink));
            
            fperp_x1 = fperp_x1 - fspring_xn - fdamper_xn;
            fperp_xn = fspring_xn + fdamper_xn;
            
            %yn
            fspring_yn = Ksp*cos(th1)*(1-dn/df);
            fdamper_yn = b/df*( u1pulink*(vf*sin(thf)-v1*sin(th1)) + cos(th1)*(vf*unulink-v1*u1ulink));
            
            fperp_y1 = fperp_y1 - fspring_yn -  fdamper_yn;
            fperp_yn = fspring_yn + fdamper_yn;
            
            %vn
            fperp_vn = b*u1pulink*unulink;
            fperp_v1 = fperp_v1 -b*u1pulink*u1ulink;
            
            %theta n
            fperp_thn = b*vf*u1pulink*unpulink;
            fperp_th1 = fperp_th1 - Ksp*(1-dn/df)*u1ulink - b*vf*unulink*u1ulink + b*v1*( u1ulink^2 - u1pulink^2 );
            
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
        
        F(4, 1) = beta*fperp_x1;
        F(4, 2) = beta*fperp_y1;
        F(4, 3) = beta*fperp_v1;
        F(4, 4) = beta*fperp_th1;
        
        %% QKALMAN
        %q1 geldi simple'a cekildi.
        
        %% PhiK
        phiK = eye(N)+F*ts;
        
        %% StatePropagate
        for ir = 1:nRobots
            if ir == 1
                xpre(1 ) = xk(1 ) + xk(3 )*cos(xk(4 ))*ts;
                xpre(2 ) = xk(2 ) + xk(3 )*sin(xk(4 ))*ts;
                xpre(3 ) = xk(3 ) + acomArr(ik)*ts*1;
                xpre(4 ) = xk(4 ) + ortdotArr(ik)*ts*1;
            else
                xpre(1 + n*(ir-1)) = xk(1 + n*(ir-1)) + xk(3 + n*(ir-1))*cos(xk(4 + n*(ir-1)))*ts;
                xpre(2 + n*(ir-1)) = xk(2 + n*(ir-1)) + xk(3 + n*(ir-1))*sin(xk(4 + n*(ir-1)))*ts;
                xpre(3 + n*(ir-1)) = xk(3 + n*(ir-1));
                xpre(4 + n*(ir-1)) = xk(4 + n*(ir-1));
            end
        end
        
        %% covariance propogation
        Mk = phiK*P_k*phiK' + Qk;
        
        %% calculate Kalman gain
        K = Mk*H'*(H*Mk*H' + Rk)^-1;
        
        %% create new estimation
        xk = xpre + K*(meas - H*xpre);
        
        %                 for ir = 2:nRobots
        %                    if(xk(3 + (ir-1)*n) < 0)
        %                        xk(3 + (ir-1)*n) = -xk(3 + (ir-1)*n);
        %                        xk(4 + (ir-1)*n) = xk(4 + (ir-1)*n) + pi;
        %                        xk(4 + (ir-1)*n) = wrapToPi(xk(4 + (ir-1)*n));
        %                    end
        %         %            xk(4 + (ir-1)*n) = wrapToPi(xk(4 + (ir-1)*n));
        %                 end
        
        %% update cov
        P_k = (eye(N)-K*H)*Mk;
        
        %% save
        xks{ik} = xk;
        Pks{ik} = P_k;
        
        %% yeni force hesaplama
        
        x1 = xk(1);
        y1 = xk(2);
        v1 = xk(3);
        th1 = xk(4);
        
        u1 = [cos(th1);sin(th1)];
        u1p = [-sin(th1);cos(th1)];
        
        fparallel = 0;
        fperp = 0;
        
        for ir = 2:nRobots
            
            xf = xk(1 + (ir-1)*n);
            yf = xk(2 + (ir-1)*n); %xfocus,yfocus gibi
            vf = xk(3 + (ir-1)*n);
            thf = xk(4 + (ir-1)*n);
            df = sqrt( (xf - xk(1))^2 + (yf - xk(2))^2 );
            
            %             dfMeas = sqrt( (meas(4 + (ir-2)*2))^2 + (meas(5 + (ir-2)*2))^2 );
            dfMeas = df;
            
            ulink = [xf-x1; yf-y1]/df;
            un = [cos(thf); sin(thf)];
            unp = [-sin(thf); cos(thf)];
            
            u1ulink = dot(u1,ulink);
            u1pulink = dot(u1p,ulink);
            unulink = dot(un,ulink);
            unpulink = dot(unp,ulink);
            
            fparallel = fparallel + Ksp*(dfMeas-dn)*u1ulink + b*(vf*unulink-v1*u1ulink)*u1ulink;
            fperp = fperp + Ksp*(dfMeas-dn)*u1pulink + b*(vf*unulink-v1*u1ulink)*u1pulink;
            
            
        end
        
        % OBSTACLE,
        agentFxObs = 0;
        agentFyObs = 0;
        
        
        %%%% rectangular obs.
        
        for j = 1:size(obsList,1)
            xObs = x1-obsList(j,1);
            yObs = y1-obsList(j,2);
            rObs = sqrt( xObs.^2 + yObs.^2 );
            fObs = obsK./(rObs-obsList(j,3)).^2.*(rObs<(obsList(j,3)+obsD));
            
            agentFxObs = agentFxObs + fObs.*xObs./rObs;
            agentFyObs = agentFyObs + fObs.*yObs./rObs;
        end
        
        fparallel_obs   = [agentFxObs, agentFyObs]*u1;
        fperp_obs       = [agentFxObs, agentFyObs]*u1p;
        
        fparallel   = fparallel*1 + fparallel_obs;
        fperp       = fperp*1 + fperp_obs;
        
        %% command
        acomArr(ik)     = Kacc*(vel + fparallel*alpha - v1);
        ortdotArr(ik)   = beta*fperp;
        
        %% log
        if ik==iFocal
            xkArr(:,i)      = xk;
            xpreArr(:,i)      = xpre;
            xrArr(:,i) = xr;
            measArr(:,i) = meas;
            Parr (:,i)      = diag(P_k);
        end
    end
    
end
toc

% plotSim2