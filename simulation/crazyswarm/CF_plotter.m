M = dlmread('logKalman ().txt');
nr = 6;
iFocal = 4;
ts = 50e-3 *1;

posx = M(:,1:6);
posy = M(:,7:12);
ort = M(:,13:18);
vel = M(:,19:24);
xk = M(:,25:48);
pk = M(:,49:72);

%% trajectory
figure,hold all
for i = 1:nr
    plot(posx(:,i),posy(:,i),'linewidth',2)
end
grid on
daspect([1 1 1])

%% velocity - komut gerceklestirilmesi
figure, hold all
plot(vel)
plot( sqrt( diff(posx).^2 + diff(posy).^2 ) / ts )
grid

%% orientation
figure, hold all
plot(ort*180/pi)
plot( atan2d( diff(posy) , diff(posx) ) )
grid

%% posx estim
figure,hold all
plot(posx(:,iFocal))
plot(xk(:,1),'o')
legend meas est
grid

%% posy estim
figure,hold all
plot(posy(:,iFocal))
plot(xk(:,2),'o')
legend meas est
grid

%% posy estim neighbor
figure,hold all
plot(posy(:,1))
plot(xk(:,2+4),'o')
legend meas est
grid

%% vel estim
figure,hold all
plot(vel(:,iFocal))
plot( sqrt( diff(posx(:,iFocal)).^2 + diff(posy(:,iFocal)).^2 ) / ts )
plot(xk(:,3),'o')
legend meas est
grid

%% vel estim neighbor
figure,hold all
plot(vel(:,1))
% plot( sqrt( diff(posx(:,1)).^2 + diff(posy(:,1)).^2 ) / ts )
plot( sqrt( gradient(posx(:,1)).^2 + gradient(posy(:,1)).^2 ) / ts )
plot(xk(:,3+4),'o')
legend meas est
grid

%% ort estim
figure,hold all
plot(ort(:,iFocal)*180/pi)
plot( atan2d( diff(posy(:,iFocal)) , diff(posx(:,iFocal)) ) )
plot(xk(:,4)*180/pi,'o')
legend meas est
grid

%% ort neighbor estim
figure,hold all
idxN = 2;
plot(ort(:,idxN))
plot( atan2( diff(posy(:,idxN)) , diff(posx(:,idxN)) ) )
plot(xk(:,4+4*idxN),'o')
legend meas est
grid

%% posx error
figure,hold all
plot(posx(:,iFocal)-xk(:,1),'o')
grid

%% posy error
figure,hold all
plot(posy(:,iFocal)-xk(:,2),'o')
grid

%%