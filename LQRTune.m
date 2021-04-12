function [Q,R,PD] = LQRTune()
clc; clear all;
% Get some parallel thread workers ready to do some work

% pool = parpool(4);
% pctRunOnAll warning('off','all')

% Setup some parameters for the genetic algorithm


numIterations = 15;
numParams = 15;
popSize = 20;
population = zeros(popSize,numParams,numIterations);
paramRange = [0, 100;
              0, 100;
              0, 100;
              0, 50;
              0, 50;
              0, 50;
              0, 1;
              0, 1;
              0, 1
%               0, 50
%               0, 5
%               0, 50
%               0, 5
%               0, 5
%               0, 1
              25, 25.5
              2.4, 2.5
              25, 25.5
              2.4, 2.5
              1.4, 1.6
              0.05, 0.15];
          
% paramRange = [zeros(numParams,1) 10000*ones(numParams,1)];
individualFitness = zeros(1,popSize);
genFitness = zeros(numIterations,popSize);
iterationMinFitness = zeros(1,numIterations);

% Set population quantities (must sum to popSize)
numElite = 2;   % Elite Children
numRecom = 10;   % Recombination Children
numMut = 5;     % Mutation Children
numRand = 3;    % Randomly Generated Children

if sum([numElite,numRecom,numMut,numRand])~=popSize
    error('There is a mistake with your population set up')
end

% Set number of parent[Qs to be picked for recombination
numParents = 5;

% %Generate random population within given parameter ranges
% population(1,:,1) = [20.3521   67.3492   82.5232    5.0364   38.4360   43.2765 0.8478    0.8362    0.4305];
% population(2,:,1) = [23.0406   18.7731  155.4757    5.1041   39.2045   44.4882 0.8744    0.8512    0.7455];
% population(3,:,1) = [827.0183  964.1610  692.7486   27.6830   10.4435    6.2735 0.1963    0.3074    0.0831];
% population(4,:,1) = [22.3566   44.6603  120.0432    5.1038   39.1358   44.4467 0.8647    0.8506    0.8037];
% population(5,:,1) = [677.8466  885.3352  874.8402   29.0917   17.9303   21.9967 0.1563    0.2512    0.1626];
% population(6,:,1) = [200.2163  114.9792   50.1092   17.5759   11.9471   29.3049 0.3388    0.9212    0.6934];
% population(1,:,1) = [85.9296  111.8189   82.2871    6.0150   22.0982   37.4103 0.4099    0.8274    0.6708]; %best for no noise
% population(2,:,1) = [23.0406   17.6794  155.4757    5.1041   39.2045   44.4882 0.8742    0.8512    0.7455]; % best for noise (0.1s)
for i = 1:numParams
    range = paramRange(i,2)-paramRange(i,1);
    for j = 1:popSize
        population(j,i,1) =  range.*rand(1) + paramRange(i,1);
    end
end

%Run simulation for each individual in the population
tic
for generation = 1:numIterations
    fprintf('\nGeneration %d',generation);
    
    Kq1s = population(:,1,generation);       %Parameter q1
    Kq2s = population(:,2,generation);       %Parameter q2
    Kq3s = population(:,3,generation);       %Parameter q3  
    Kq4s = population(:,4,generation);       %Parameter q4
    Kq5s = population(:,5,generation);       %Parameter q5
    Kq6s = population(:,6,generation);       %Parameter q6 
    Kr1s = population(:,7,generation);       %Parameter r1
    Kr2s = population(:,8,generation);       %Parameter r2  
    Kr3s = population(:,9,generation);       %Parameter r3
    Kp_phis = population(:,10,generation);
    Kd_phis = population(:,11,generation);
    Kp_thetas = population(:,12,generation);
    Kd_thetas = population(:,13,generation);
    Kp_psidots = population(:,14,generation);
    Kd_psidots = population(:,15,generation);
%     parfor i = 1:popSize
    for i = 1:popSize
        %Run the simulation for this individual
        Kq1 = Kq1s(i);
        Kq2 = Kq2s(i);
        Kq3 = Kq3s(i);
        Kq4 = Kq4s(i);
        Kq5 = Kq5s(i);
        Kq6 = Kq6s(i);
        Kr1 = Kr1s(i);
        Kr2 = Kr2s(i);
        Kr3 = Kr3s(i);
        PD.Kp_phi = Kp_phis(i);
        PD.Kd_phi = Kd_phis(i);
        PD.Kp_theta = Kp_thetas(i);
        PD.Kd_theta = Kd_thetas(i);
        PD.Kp_psidot = Kp_psidots(i);
        PD.Kd_psidot = Kd_psidots(i);
        p = 0;
        % Run the simulation and quantify results (or just check the step response of the plant)
        Q = diag([Kq1,Kq2,Kq3,Kq4,Kq5,Kq6,1]);
        R = diag([Kr1,Kr2,Kr3,1]);
        %PD = [Kp_phi,Kd_phi,Kp_theta,Kd_theta,Kp_psidot,Kd_psidot];
        for trial = 1
            Params = ret_param(Q,R,PD,trial);
            try 
                results = sim('models/LQR_3_HeadlessPositionControllerVariableQuadDynamicsMdl.slx','SrcWorkspace','current',...
                     'SaveFormat','Dataset');
                quad_states = results.get('States');
                
                control_com = results.get('Control_Command');
                
                traj = results.get('DesiredTrajectory');
                
%                 disp(diag(Q)');
%                 disp(diag(R)');
                p = p + fit_fn(quad_states,traj,control_com);
            catch ME
                ME.cause{:}
                p = p + inf;
            end
        end
        %Calculate the cost function from the simulation
        p;
        individualFitness(i) = p; 
        fprintf('.');
    end
    genFitness(generation,:) = individualFitness;
    iterationMinFitness(generation) = min(individualFitness);
    fprintf('Best fitness: %f',iterationMinFitness(generation));
    
    % Select elite children for next generation
    [v,fitIndex] = sort(individualFitness);
    eliteChildren = population(fitIndex(1:numElite),:,generation);
    parentPool = population(:,:,generation);
    parentPool(fitIndex(1:numElite),:) = [];
    
    % Select parents to compete
    individualFitness(fitIndex(1:numElite)) = [];
    fitnessValues = individualFitness;
    parentIndex= zeros(1,numParents);

    % Select parents by group tournaments
    K = 3;
    for j = 1:numParents-numElite
        randomPick = randperm(popSize-numElite-j+1);
        contestants = randomPick(1:K);
        parentIndex(j) = find(individualFitness == min(fitnessValues(contestants)), 1 );
        % Remove selection from population
        fitnessValues(find(fitnessValues==individualFitness(parentIndex(j)),1)) = [];
    end
    % Add elite children into the parent pool
    parentIndex(j+1:end) = fitIndex(1:numElite);
   
    recomChildren = zeros(numRecom+numMut,numParams);
    %%% Modified BLX-alpha crossover recombination
    for j = 1:numRecom+numMut
        randomPick3 = randperm(numParents);
        parents = parentIndex(randomPick3(1:2));
        for k = 1:numParams
            x1x2 = [population(parents(1),k,generation),population(parents(2),k,generation)];
            xMin = min(x1x2);
            xMax = max(x1x2);
            ak = paramRange(k,1);
            bk = paramRange(k,2);
            alpha = abs(diff(x1x2))/diff(paramRange(k,:));
            I = min([xMin-ak,bk-xMax]);
            newRange = [xMin - I*alpha,xMax + I*alpha];
            flag = 0;
            if newRange(1)<ak
                flag = 1;
                newRange(1) = ak;
            end
            if  newRange(2)>bk
                flag = 1;
                newRange(2) = bk;
            end
            newInterval = diff(newRange);
            recomChildren(j,k) = newInterval*rand() + newRange(1);
        end
    end
    if flag
        warning('Some parameters were out of specified ranges.\n All such values were restricted to range limits')
    end

    % Non-uniform mutation of children
    mutChildren = recomChildren(1:numMut,:);
    numGen = 2;  % Number of genetic components to mutate within each gene
    b = 1.5;  % Arbitrary exponent to drive late-generation mutations to 0
    for j = 1:numMut
        coinFlip = round(rand(1,numGen));
        lambdaM = rand(1,numGen);
        randomPickM = randperm(numParams);
        mutGen = randomPickM(1:numGen);
        for k = 1:numGen
            if coinFlip(k)
                mutChildren(j,mutGen(k)) = paramRange(mutGen(k),1)+...
                    (recomChildren(j,mutGen(k))-paramRange(mutGen(k),1))...
                    *(1-lambdaM(k)*(1-generation/numIterations)^b);
            else
                mutChildren(j,mutGen(k)) = paramRange(mutGen(k),2)-...
                    (paramRange(mutGen(k),2)-recomChildren(j,mutGen(k)))...
                    *(1-lambdaM(k)*(1-generation/numIterations)^b);
            end
        end
    end
    
    %Generate random children
    if numRand
        randChildren = zeros(numRand,numParams);
        for i = 1:numParams
            range = paramRange(i,2)-paramRange(i,1);
            for j = 1:numRand
                randChildren(j,i) =  range.*rand(1) + paramRange(i,1);
            end
        end
    end

    % Build next generation
    sumChildren = sum([numRecom,numElite,numMut]);
    if generation<numIterations
        population(1:sumChildren,:,generation+1) = [eliteChildren;
            recomChildren(numMut+1:end,:);
            mutChildren];
        if numRand
            population(sumChildren+1:end,:,generation+1) = randChildren;
        end
    end
end
toc
% delete(pool)

[~,loc] = min(genFitness(end,:));
bestGains = population(loc,:,end);
Kq1 = bestGains(1); Kq2 = bestGains(2); Kq3 = bestGains(3);
Kq4 = bestGains(4); Kq5 = bestGains(5); Kq6 = bestGains(6);
Kr1 = bestGains(7); Kr2 = bestGains(8); Kr3 = bestGains(9);
Kp_phi = bestGains(10); Kd_phi = bestGains(11);
Kp_theta = bestGains(12); Kd_theta = bestGains(13);
Kp_psidot = bestGains(14); Kd_psidot = bestGains(15);
Q = diag([Kq1,Kq2,Kq3,Kq4,Kq5,Kq6,1]);
R = diag([Kr1,Kr2,Kr3,1]);
PD = [Kp_phi,Kd_phi,Kp_theta,Kd_theta,Kp_psidot,Kd_psidot];
end
