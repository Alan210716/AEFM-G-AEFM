clc % 清空命令行窗口
clear % 从当前工作区中删除所有变量
close all % 删除所有可见图形

fileNames = {'L_47_0.2_30.mat'};
Init_time = inf(32,41);
probs = ones(32,41);
Init_t = 0;
cyclebest = zeros(32,25);
result_record = zeros(32,25,500);
time_record = zeros(32,25,500);
test_record = zeros(32,2);
Iter_record = zeros(32,25);
grp_Array = {};
save_t = zeros(10,1);

tic % 保存当前时间
tem_toc = 0;
% Iter_save = [93, 125, 100];

for Init_k = 1
    for cycle_num = 1:25
        cycle = cycle_num;
        feature('JIT','off');
        
        %% 参数初始化
        filename = fileNames{Init_k};
        load(filename);
        
        Fai = (CityNum+1)*increase*mean(Demand)/(M*ability);
        cF = 4;
        origin_M = M;
        ar = 7;
        save_rate = 0.45;
        best_rate = 0.45;
        muta_rate = 0;
        Taumax = inf;
        Taumin = 0.015;
        omega = 4;
        aggrenum = 2;
        MaxIter = 100;
        probs(Init_k,:) = ones(1,41);
        ttltime = zeros(AntNum,1);
        
        Init_Distance = Distance;
        Init_Demand = Demand;
        Init_City = City;
        
        Eta_d = ones(CityNum*3);
        Eta_s = 1./Distance;
        mintime = inf;
        
        %% 迭代寻找最佳路径
        Iter = 1;
        
        while Iter <= MaxIter
            if toc - tem_toc < 2/3 * MaxIter
                grp = assignGrpsWithCDF(origin_M, probs(Init_k,:), aggrenum);
            else
                grp = best_grp;
            end
            
            grps = sort(grp, 'descend');
            M = length(grps);
            
            for a1 = 1:M
                robot(a1).abil = grps(a1) * ability;
                robot(a1).Tau1 = zeros(3*CityNum);
            end
            
            countgrp = 0;
            
            %% 逐个路径选择
            for a1 = 1:M
                robot(a1).Tau1 = zeros(3*CityNum);
                for a2 = 1:grps(a1)
                    countgrp = countgrp + 1;
                    robot(a1).Tau1 = robot(a1).Tau1 + robot(countgrp).Tau;
                end
            end
            
            for i = 1:AntNum
                flagWrong = 0;
                TSProute = 2:CityNum+1;
                Distime = 0;
                NewCity = 0;
                Distance = Init_Distance;
                Demand = Init_Demand;
                City = Init_City;
                
                % 智能体状态初始化
                for a = 1:M
                    robot(a).Ce = 0;
                    robot(a).target = 0;
                    robot(a).departure = [];
                    robot(a).remainway = 0;
                    robot(a).P = [];
                    robot(a).temtime = [0];
                    robot(a).DisTraveled = 0;
                    robot(a).delivery = 0;
                    robot(a).VRProute = [1];
                    robot(a).delete = 1;
                end
                
                % 城市状态初始化
                for a = 1:CityNum+1
                    city(a).demand = Demand(a);
                    city(a).VaryDemand = Demand(a);
                    city(a).SaveDemand = Demand(a);
                    city(a).ttability = 0;
                    city(a).NR = 0;
                    city(a).finishtime = 0;
                    city(a).onway = 0;
                end
                
                [CitySum,~] = size(City);
                if CitySum > CityNum+1
                    City(CityNum+2:CitySum,:) = [];
                    city(CityNum+2:CitySum) = [];
                end
                
                % 各个机器人初始的目标城市
                P0 = zeros(M,CityNum);
                Eta0 = zeros(M,CityNum);
                Pc0 = zeros(M,CityNum);
                Eta_d0 = zeros(M,CityNum);
                
                for k1 = 1:M
                    for k = 1:CityNum
                        incre_v = city(k+1).demand * increase / ability;
                        sol_v = city(k+1).NR + city(k+1).onway;
                        
                        if sol_v/incre_v > omega
                            Eta_d0(k1,k) = 1 - sol_v/(incre_v+sol_v);
                        elseif sol_v/incre_v <= 1+(omega-1)*Fai/(Fai+cF) && Fai > cF
                            Eta_d0(k1,k) = 1 + sol_v/(incre_v+sol_v);
                        elseif sol_v/incre_v <= 1 && Fai <= cF
                            Eta_d0(k1,k) = 1 + sol_v/(incre_v+sol_v);
                        else
                            Eta_d0(k1,k) = 1;
                        end
                        
                        Eta0(k1,k) = robot(k1).Tau1(1,k+1) * Eta_d0(k1,k) * Demand(k+1) * Eta_s(1,k+1)^ar;
                    end
                    
                    P0(k1,:) = Eta0(k1,:) / sum(Eta0(k1,:));
                    Pc0(k1,:) = cumsum(P0(k1,:));
                    TargetIndex = find(Pc0(k1,:) >= rand);
                    robot(k1).target = TSProute(TargetIndex(1));
                    robot(k1).remainway = Distance(1, robot(k1).target);
                    city(robot(k1).target).onway = city(robot(k1).target).onway + robot(k1).abil/ability;
                end
                
                %% 开辟新的路径
                while ~isempty(TSProute)
                    % 找最近的产生到达或离开事件的智能体编号
                    i1 = 0;
                    temtime = 0;
                    pretime = Inf;
                    dis = 0;
                    
                    for a = 1:M
                        if robot(a).Ce == 0 % 转移状态
                            temtime = robot(a).remainway / v;
                            if temtime < pretime
                                num = [];
                                pretime = temtime;
                                i1 = 1;
                                num(i1) = a;
                            elseif temtime == pretime
                                i1 = i1 + 1;
                                num(i1) = a;
                            end
                        else % 工作状态
                            if robot(a).Ce == 1 && all(robot(a).P == 0)
                                temtime = Inf;
                            else
                                temtime = city(robot(a).target).VaryDemand / (city(robot(a).target).ttability - city(robot(a).target).demand * increase);
                            end
                            
                            if temtime < pretime && city(robot(a).target).ttability > city(robot(a).target).demand * increase
                                num = [];
                                pretime = temtime;
                                i1 = 1;
                                num(i1) = a;
                            elseif temtime == pretime
                                i1 = i1 + 1;
                                num(i1) = a;
                            end
                        end
                    end
                    
                    %% 经过pretime时间之后
                    a = num;
                    Distime = Distime + pretime;
                    
                    % 更新赶路状态智能体剩余里程
                    for k = 1:M
                        robot(k).remainway = robot(k).remainway - v * pretime;
                        if robot(k).remainway < 1e-10
                            robot(k).remainway = 0;
                        end
                    end
                    
                    % 更新工作状态智能体耗电量
                    for k = 1:M
                        if robot(k).Ce > 1
                            robot(k).delivery = robot(k).delivery + robot(k).abil * pretime;
                        end
                    end
                    
                    % 更新城市时变需求和已解决需求
                    for k = 1:length(TSProute)
                        city(TSProute(k)).VaryDemand = city(TSProute(k)).VaryDemand + pretime * increase * city(TSProute(k)).demand - city(TSProute(k)).ttability * pretime;
                        if city(TSProute(k)).VaryDemand < 1e-10
                            city(TSProute(k)).VaryDemand = 0;
                        end
                        city(TSProute(k)).SaveDemand = city(TSProute(k)).SaveDemand + pretime * increase * city(TSProute(k)).demand;
                    end
                    
                    % 发生事件智能体和城市状态更新
                    for j = 1:length(a)
                        if ~(pretime == 0 && robot(a(j)).Ce == 1)
                            robot(a(j)).temtime = [robot(a(j)).temtime Distime];
                        end
                        
                        if robot(a(j)).Ce == 0 % 上一时刻赶路，即将到达城市
                            % 城市状态更新
                            city(robot(a(j)).target).ttability = city(robot(a(j)).target).ttability + robot(a(j)).abil;
                            city(robot(a(j)).target).NR = city(robot(a(j)).target).NR + robot(a(j)).abil/ability;
                            if robot(a(j)).target ~= 1
                                city(robot(a(j)).target).onway = city(robot(a(j)).target).onway - robot(a(j)).abil/ability;
                            end
                            
                            % 智能体状态更新
                            robot(a(j)).Ce = robot(a(j)).target;
                            robot(a(j)).remainway = 0;
                            robot(a(j)).DisTraveled = robot(a(j)).DisTraveled + Distance(robot(a(j)).delete(end), robot(a(j)).target);
                        else % 上一时刻完成任务，即将赶路
                            % 城市状态更新
                            city(robot(a(j)).Ce).finishtime = Distime;
                            city(robot(a(j)).target).pathlist = [];
                            city(robot(a(j)).Ce).ttability = 0;
                            city(robot(a(j)).Ce).NR = 0;
                            
                            % 充电加燃料后智能体状态更新
                            if robot(a(j)).Ce == 1
                                robot(a(j)).DisTraveled = 0;
                                robot(a(j)).delivery = 0;
                            end
                            
                            if robot(a(j)).target ~= 1
                                robot(a(j)).delete = [robot(a(j)).delete, robot(a(j)).target];
                            end
                            
                            % TSP路径中排除已安排的城市
                            TSProute = setdiff(TSProute, robot(a(j)).delete);
                            robot(a(j)).VRProute = [robot(a(j)).VRProute, robot(a(j)).delete(end)];
                            
                            if isempty(TSProute)
                                for k = 1:M
                                    robot(k).delete = 1;
                                end
                                continue;
                            end
                            
                            % 对空闲智能体更新概率矩阵
                            robot(a(j)).P = TSProute;
                            count = 0;
                            SaveTSP = [];
                            
                            for k = 1:length(TSProute)
                                incre_v = city(TSProute(k)).demand * increase / ability;
                                sol_v = city(TSProute(k)).NR + city(TSProute(k)).onway;
                                
                                if TSProute(k) <= CityCount
                                    count = count + 1;
                                    SaveTSP = [SaveTSP, TSProute(k)];
                                end
                                
                                if Distance(robot(a(j)).delete(end), TSProute(k))/v > city(TSProute(k)).VaryDemand/(city(TSProute(k)).ttability - city(TSProute(k)).demand * increase) && city(TSProute(k)).ttability > city(TSProute(k)).demand * increase
                                    Eta_d(robot(a(j)).delete(end), TSProute(k)) = 0;
                                elseif sol_v/incre_v > omega
                                    Eta_d(robot(a(j)).delete(end), TSProute(k)) = 1 - sol_v/(incre_v+sol_v);
                                elseif sol_v/incre_v <= 1 && Fai <= cF
                                    Eta_d(robot(a(j)).delete(end), TSProute(k)) = 1 + sol_v/(incre_v+sol_v);
                                elseif sol_v/incre_v <= 1+(omega-1)*Fai/(Fai+cF) && Fai > cF
                                    Eta_d(robot(a(j)).delete(end), TSProute(k)) = 1 + sol_v/(incre_v+sol_v);
                                else
                                    Eta_d(robot(a(j)).delete(end), TSProute(k)) = 1;
                                end
                            end
                            
                            for k1 = 1:length(TSProute)
                                robot(a(j)).P(k1) = robot(a(j)).Tau1(robot(a(j)).delete(end), TSProute(k1)) * Eta_d(robot(a(j)).delete(end), TSProute(k1)) * city(TSProute(k1)).demand * Eta_s(robot(a(j)).delete(end), TSProute(k1))^ar;
                            end
                            
                            % 根据概率矩阵找目标
                            if all(robot(a(j)).P == 0)
                                robot(a(j)).target = 1;
                            else
                                robot(a(j)).P = robot(a(j)).P / sum(robot(a(j)).P);
                                Pc = cumsum(robot(a(j)).P);
                                TargetIndex = find(Pc >= rand);
                                
                                if isempty(TargetIndex) == 0 && length(robot(a(j)).VRProute) < 2 * CityNum
                                    robot(a(j)).target = TSProute(TargetIndex(1));
                                else
                                    flagWrong = 1;
                                    break;
                                end
                                robot(a(j)).target = TSProute(TargetIndex(1));
                            end
                            
                            avaicount = 0;
                            availength = length(TSProute);
                            Record = 0;
                            Save = 0;
                            
                            % 返回配置中心处理
                            if robot(a(j)).delete(end) ~= 1 && robot(a(j)).target == 1
                                robot(a(j)).target = 1;
                                robot(a(j)).departure = [];
                                robot(a(j)).remainway = Distance(robot(a(j)).Ce, 1);
                                robot(a(j)).delete = 1;
                            elseif robot(a(j)).DisTraveled + Distance(robot(a(j)).delete(end), robot(a(j)).target) + Distance(robot(a(j)).target, 1) > Travelcon
                                avaicount = 0;
                                if availength > 0
                                    for k = 1:availength
                                        if robot(a(j)).DisTraveled + Distance(robot(a(j)).delete(end), TSProute(availength)) + Distance(TSProute(availength), 1) <= Travelcon && city(TSProute(availength)).VaryDemand/(city(TSProute(availength)).ttability - city(TSProute(availength)).demand * increase) > Distance(robot(a(j)).delete(end), TSProute(availength))/v
                                            Record = robot(a(j)).Tau1(robot(a(j)).delete(end), TSProute(k)) * Eta_d(robot(a(j)).delete(end), TSProute(k)) * city(TSProute(k)).demand * Eta_s(robot(a(j)).delete(end), TSProute(k))^ar;
                                            if Record > Save
                                                Save = Record;
                                                robot(a(j)).target = TSProute(availength);
                                                robot(a(j)).remainway = Distance(robot(a(j)).Ce, robot(a(j)).target);
                                            end
                                            avaicount = avaicount + 1;
                                        end
                                    end
                                end
                                
                                if availength == 0 || avaicount == 0
                                    robot(a(j)).target = 1;
                                    robot(a(j)).departure = [];
                                    robot(a(j)).remainway = Distance(robot(a(j)).Ce, 1);
                                    robot(a(j)).delete = 1;
                                end
                                
                                if robot(a(j)).target ~= 1
                                    city(robot(a(j)).target).onway = city(robot(a(j)).target).onway + robot(a(j)).abil/ability;
                                end
                            else
                                robot(a(j)).remainway = Distance(robot(a(j)).Ce, robot(a(j)).target);
                                if robot(a(j)).target ~= 1
                                    city(robot(a(j)).target).onway = city(robot(a(j)).target).onway + robot(a(j)).abil/ability;
                                end
                            end
                            
                            if ~(all(robot(a(j)).P == 0) && robot(a(j)).Ce == 1)
                                robot(a(j)).Ce = 0;
                            end
                        end
                    end
                    
                    if flagWrong == 1
                        break;
                    end
                end
                
                if flagWrong == 1
                    ttltime(i) = inf;
                    flagWrong = 0;
                    continue;
                end
                
                %% 路径处理及记录最优路径
                for a = 1:M
                    fillwithones = linspace(1, 1, CityNum*2+1 - length(robot(a).VRProute));
                    fillwithtimes = linspace(Inf, Inf, CityNum*4+2 - length(robot(a).temtime));
                    robot(a).VRProute = [robot(a).VRProute, fillwithones];
                    robot(a).temtime = [robot(a).temtime, fillwithtimes];
                    Population(a,:,i) = robot(a).VRProute;
                end
                
                if Distime < mintime
                    mintime = Distime;
                    bestcity = city;
                    bestCity = City;
                end
                
                ttltime(i) = Distime;
                [min_value, min_index] = min(ttltime(1:i));
                
                if i == min_index
                    for a = 1:M
                        robot(a).besttime = robot(a).temtime;
                    end
                    save_p = probs;
                    save_grp = grps;
                    p_index = min_index;
                    save_M = M;
                end
            end
            
            %% 存储历史最优信息
            if Iter == 1
                [min_Length, min_index] = min(ttltime);
                MinTime(Iter) = min_Length;
                best_Population = Population;
                best_ttltime = ttltime;
                best_index = min_index;
                best_grp = grps;
                best_p = probs;
                best_M = M;
                cellgrp{Init_k, cycle_num} = grps;
            else
                [min_Length, min_index] = min(ttltime);
                MinTime(Iter) = min(MinTime(Iter-1), min_Length);
                
                if min_Length == MinTime(Iter)
                    for a = 1:M
                        bestind(a,:) = Population(a,:,min_index);
                        besttime(a,:) = robot(a).besttime;
                    end
                    best_Population = Population;
                    best_ttltime = ttltime;
                    best_index = min_index;
                    best_grp = grps;
                    best_p = probs;
                    best_M = M;
                    cellgrp{Init_k, cycle_num} = grps;
                end
            end
            
            %% 更新算法参数
            grpcount = 0;
            if Iter ~= 1
                for a = 1:save_M
                    for a1 = 1:save_grp(a)
                        grpcount = grpcount + 1;
                        robot(grpcount).Delta_Tau = zeros(CityNum*3);
                        for j = 1:2*CityNum
                            robot(grpcount).Delta_Tau(Population(a,j,min_index), Population(a,j+1,min_index)) = robot(a).Delta_Tau(Population(a,j,min_index), Population(a,j+1,min_index)) + (mintime/ttltime(min_index)) * save_rate * exp(-Iter*Taumin);
                        end
                        robot(grpcount).Tau = (robot(grpcount).Tau + robot(grpcount).Delta_Tau);
                    end
                end
                
                grpcount = 0;
                for a = 1:best_M
                    for a1 = 1:best_grp(a)
                        grpcount = grpcount + 1;
                        robot(grpcount).Delta_Tau = zeros(CityNum*3);
                        for j = 1:CityNum
                            robot(grpcount).Delta_Tau(best_Population(a,j,best_index), best_Population(a,j+1,best_index)) = robot(a).Delta_Tau(best_Population(a,j,best_index), best_Population(a,j+1,best_index)) + best_rate;
                        end
                        robot(grpcount).Tau = (robot(grpcount).Tau + robot(grpcount).Delta_Tau);
                    end
                end
                
                for a = 1:origin_M
                    robot(a).Tau = robot(a).Tau / (1 + save_rate);
                end
            end
            
            % 更新概率
            for a = 1:length(save_grp)
                probs(Init_k, save_grp(a)) = probs(Init_k, save_grp(a)) + save_rate * (mintime / ttltime(p_index)) * save_grp(a) / max(save_grp);
            end
            
            for a = 1:length(best_grp)
                probs(Init_k, best_grp(a)) = probs(Init_k, best_grp(a)) + best_rate * best_grp(a) / max(best_grp);
            end
            
            probs(Init_k,:) = probs(Init_k,:) / (1 + save_rate);
            
            for a = 1:length(probs(Init_k,:))
                if probs(Init_k,a) > 2
                    probs(Init_k,a) = 2;
                elseif probs(Init_k,a) < 0.05
                    probs(Init_k,a) = 0.05;
                end
            end
            
            for a = 1:origin_M
                for j = 1:2*CityNum
                    for j1 = j:2*CityNum+1
                        if robot(a).Tau(j,j1) < Taumin
                            robot(a).Tau(j,j1) = Taumin;
                            if rand <= muta_rate
                                robot(a).Tau(j,j1) = 1;
                            end
                        end
                        
                        if robot(a).Tau(j1,j) < Taumin
                            robot(a).Tau(j1,j) = Taumin;
                            if rand <= muta_rate
                                robot(a).Tau(j,j1) = 1;
                            end
                        end
                        
                        if robot(a).Tau(j,j1) > Taumax
                            robot(a).Tau(j,j1) = Taumax;
                            if rand <= muta_rate
                                robot(a).Tau(j,j1) = 1;
                            end
                        end
                        
                        if robot(a).Tau(j1,j) > Taumax
                            robot(a).Tau(j1,j) = Taumax;
                            if rand <= muta_rate
                                robot(a).Tau(j,j1) = 1;
                            end
                        end
                    end
                end
            end
            
            %% 显示此代信息
            fprintf('Iteration = %d, Min Time = %.2f\n', Iter, MinTime(Iter))
            result_record(Init_k, cycle_num, Iter) = MinTime(Iter);
            
            %% 更新迭代次数
            Iter = Iter + 1;
        end
        
        tem_toc = toc;
        
        %% 找出历史最短距离和对应路径
        grp_Array{cycle_num} = best_grp;
        mindisever = MinTime(Iter);
        disp('-------------------------------------------------------------')
        toc
        fprintf('Total time= %s \n', num2str(mindisever))
        TextOutput_2(Distance, bestind)
        disp('-------------------------------------------------------------')
        pause(0.2);
        
        cyclebest(Init_k, cycle_num) = mindisever;
        Iter_record(Init_k, cycle_num) = Iter;
    end
    
    % 求均值和标准差
    test_record(Init_k,1) = mean(cyclebest(Init_k,:));
    test_record(Init_k,2) = std(cyclebest(Init_k,:),1);
    
    Init_t = toc;
    if Init_k > 1
        save_t(Init_k) = toc - save_t(Init_k-1);
    else
        save_t(Init_k) = toc;
    end
end