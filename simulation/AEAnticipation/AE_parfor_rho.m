% load SRC16 % includes connection matrices
% load m1m2_16_all.mat
nAgents = 1141;
mLen = 3306;
load conn_matrix_str_hex_20

rng('shuffle')                          % shuffling the rng
mainSeed = randi(1e9);
rng(mainSeed);

%% sweep

p_sweep = [0, 0.4, 1];                  % p values to be evaluated
Eta_sweep = [47:52];                  % Eta values, critical region
runsPerEta = 20;%8 ;                     % runs per p and Eta

iterLim = 5e5;                % simulation total steps
opts.log_ds = 100;                  % psi logging downsample value
sideLength = 20;   %sidelength                      % matrix size

%% IC
% nAgents = 3*sideLength^2-3*sideLength+1;      % number of agents
% alpha = 0.01;           % force to velocity const
% beta = 0.12;            % force to orientation const
Ts = 0.1;               % time step
% vel = 0.002;            % self propulsion
% K = -5;                 % string constant
b = -50*0;

parallelCoeff = 2.7;    % optional, used for time estimation

% IC_sim.IC_posx = repmat(1:mSize,1,mSize);                   % initial x posns
% IC_sim.IC_posy = floor((1:nAgents)/mSize - 1/mSize/10)+1;   % init y posns

% ort = randi(3600,[nAgents 1])/10/180*pi  *1;
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

IC_sim.nAgents = nAgents;

%% calc vars
nofRuns = length(Eta_sweep)*length(p_sweep)*runsPerEta;

%% parallel comp

c = gcp('nocreate');    % start matlab parallel pool
delete(c);
p=parcluster('local');
p.NumWorkers=4;
parpool(4);

%%
IC_sim_arr      = cell(nofRuns,1);
seed_arr        = cell(nofRuns,1);
ort_seed_arr    = cell(nofRuns,1);

idx3 = 1;
for idx1 = 1:runsPerEta
    for idx2 = 1:1%2
        
        for j = 1:length(Eta_sweep)
            for i = 1:length(p_sweep)
                p_str = ['p_',num2str(p_sweep(i)*10)];
                
%                                 IC_sim_arr{idx3} = IC_sim;
%                 IC_sim_arr{idx3}.linkM_comb = conn_matrix_str.(p_str).linkM_comb{idx1};
%                 IC_sim_arr{idx3}.linkM_comb_tr = conn_matrix_str.(p_str).linkM_comb_tr{idx1};
%                 IC_sim_arr{idx3}.linkIL = conn_matrix_str.(p_str).linkIL{idx1};
                IC_sim_arr{idx3}.m1 = conn_matrix_str.(p_str).m1{idx1};
                IC_sim_arr{idx3}.m2 = conn_matrix_str.(p_str).m2{idx1};
                IC_sim_arr{idx3}.IL = conn_matrix_str.(p_str).IL{idx1};
                
                if(idx1<=runsPerEta/2) %idx2<2, 1 ordered 1 disordered!
                    IC_sim_arr{idx3}.ort = zeros([1 nAgents]);
%                     IC_orientation_arr{idx3} = randi(3600,[1 nAgents])/10/180*pi;
                else
                    rng(idx1)
                    IC_sim_arr{idx3}.ort = randi(3600,[1 nAgents])/10/180*pi;
                end
                
                seed_arr{idx3} = idx1;%randi(256);
%                 ort_seed_arr{idx3} = idx1;%randi(1e9);
                
                idx3 = idx3+1;
                
            end
        end
    end
end
             

%% computation
% supply the IC to sim function and get the results.
% runs on parallel cores (parfor)

psi = zeros(nofRuns,round(iterLim / opts.log_ds));
orts = zeros(nofRuns,IC_sim.nAgents);

disp(['nofRuns: ',num2str(nofRuns)]);
parfor_progress(nofRuns);
tic

% iter_lim = opts.iter_lim;
parfor main_idx = 1:nofRuns
    
    
    [i,j,k] = ind2sub([length(p_sweep),length(Eta_sweep),runsPerEta],main_idx);
    eta_ts_rad = Eta_sweep(j) /2/180*pi*Ts;
%     rng(ort_seed_arr{main_idx});
    ort = IC_sim_arr{main_idx}.ort;
    posx2 = posx*1.0;
    posy2 = posy*1.0;
    ort2 = ort*1.0;
    psi(main_idx,:) = linkMex(IC_sim_arr{main_idx}.m1,IC_sim_arr{main_idx}.m2,IC_sim_arr{main_idx}.IL,posx2,posy2,ort2,eta_ts_rad,seed_arr{main_idx},mLen,iterLim,b)';
    parfor_progress();
    
end

psi = reshape(psi,[length(p_sweep),length(Eta_sweep),runsPerEta,round(iterLim / opts.log_ds)]);
orts = reshape(orts,[length(p_sweep),length(Eta_sweep),runsPerEta,IC_sim.nAgents]);

%% ending time
parfor_progress(0);
D2 = duration(0,0,toc);
[h2,m2,s2] = hms(D2);
disp('time elapsed: ')
disp([num2str(h2),' hours, ',num2str(m2),' minutes, ',num2str(s2),' seconds, @'])
disp(datetime('now'))

%% close parpool
% delete(apool)
c = gcp;
delete(c)

%% store the datapiece
% record the datapiece.
% datapieces can be fused with others.
data_piece.p_sweep = p_sweep;
data_piece.Eta_sweep = Eta_sweep;
data_piece.runsPerEta = runsPerEta;
data_piece.psi = single(psi);
% data_piece.orts = orts;
data_piece.seed_arr_record = cell2mat(seed_arr);
data_piece.nAgents = IC_sim.nAgents;
data_piece.mainSeed = mainSeed;
% 
% save(['data_piece',num2str(round(now*1e5)),'rho',num2str(round(rho*1e3))],'data_piece')
data_str = ['data_piece',num2str(round(now*1e5))];
% save(['data_piece',num2str(round(now*1e5))],'data_piece')
save(data_str,'data_piece')

% data_fusing
% data_all_psi_plotter2
% figure
% plot(data_all.n32.p0.(['Eta',num2str(Eta_sweep)]).psi')
% save(['data_all_K',num2str(K*10),'alpha',num2str(alpha*1e3)],'data_all');
%%
% please remove the comment if you are going to disown the process
% exit

load gong.mat;
playerObj = audioplayer(y,Fs);
start = playerObj.SampleRate * 1; 

play(playerObj,start);

