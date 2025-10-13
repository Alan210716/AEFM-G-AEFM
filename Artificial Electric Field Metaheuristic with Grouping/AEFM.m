% 多机器人任务分配算法 - 基于启发式函数
% 改进中断城市处理方式，仅使用启发函数
% 加入bestcity结构体，及时清空city和City多余元素

clc % 清空命令行窗口
clear % 从当前工作区中删除所有变量
close all % 删除所有可见图形

fileNames = {'L_47_0.2_30.mat'};
Init_t = 0;
cyclebest = zeros(29,25);
result_record = zeros(29,25,500);
time_record = zeros(29,25,500);
test_record = zeros(29,2);
Iter_record = zeros(29,25);

tic % 保存当前时间
tem_toc = 0;

for Init_k = 1
    for cycle_num = 1:25
        cycle = cycle_num;
        feature('JIT','off');
        
        %% 参数初始化
        filename = fileNames{Init_k};
        load(filename);
        
        ar = 7;
        save_rate = 0.45;
        best_rate = 0.45;
        muta_rate = 0;
        Taumax = inf;
        Taumin = 0.015;
        omega = 4;
        cF = 4;
        MaxIter = 100;
        Fai = (CityNum+1)*increase*mean(Demand)/(M*ability);
        ttltime = zeros(AntNum,1); % 预分配内存
        
        Init_Distance = Distance;
        Init_Demand = Demand;
        Init_City = City;
        
        Eta_d = ones(CityNum*3);
        Eta_s = 1./Distance; 
        mintime = inf; % 全局最优时间
        
        %% 迭代寻找最佳路径
        Iter = 1; % 迭代次数初始化
%         while(true)

        while Iter <= MaxIter
            %% 逐个路径选择
            for i = 1:AntNum
                TSProute = 2:CityNum+1; % 待解决任务地点标签
                Distime = 0; % 总时间线
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
                    robot(a).delete = 1; % 给第一次进入内while的P(k)首项
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
                if CitySum > CityNum+1 % 每次重新找路径删除中途城市
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
                        incre_v = city(k+1).demand*increase/ability;
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
                        
                        Eta0(k1,k) = robot(k1).Tau(1,k+1)*Eta_d0(k1,k)*Demand(k+1)*Eta_s(1,k+1)^ar;
                    end
                    
                    P0(k1,:) = Eta0(k1,:)/sum(Eta0(k1,:));
                    Pc0(k1,:) = cumsum(P0(k1,:));
                    TargetIndex = find(Pc0(k1,:) >= rand);
                    robot(k1).target = TSProute(TargetIndex(1));
                    robot(k1).remainway = Distance(1,robot(k1).target);
                    city(robot(k1).target).onway = city(robot(k1).target).onway + 1;
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
                            temtime = robot(a).remainway/v; % 即将到达时间
                            if temtime < pretime
                                num = [];
                                pretime = temtime;
                                i1 = 1;
                                num(i1) = a;
                            elseif temtime == pretime % 多个智能体同时到达
                                i1 = i1 + 1;
                                num(i1) = a;
                            end
                        else % 工作状态
                            if robot(a).Ce == 1 && all(robot(a).P == 0) % 已经回到仓库且不需要外出
                                temtime = Inf;
                            else
                                temtime = city(robot(a).target).VaryDemand/(city(robot(a).target).ttability-city(robot(a).target).demand*increase); % 即将离开时间
                            end
                            
                            if temtime < pretime && city(robot(a).target).ttability > city(robot(a).target).demand*increase % 预判断
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
                    a = num; % 不改变num对num遍历
                    Distime = Distime + pretime;
                    
                    % 更新赶路状态智能体剩余里程
                    for k = 1:M
                        robot(k).remainway = robot(k).remainway - v*pretime;
                        if robot(k).remainway < 1e-10
                            robot(k).remainway = 0;
                        end
                    end
                    
                    % 更新工作状态智能体耗电量
                    for k = 1:M
                        if robot(k).Ce > 1 % 原来为工作状态则耗电量增加
                            robot(k).delivery = robot(k).delivery + robot(k).abil*pretime;
                        end
                    end
                    
                    % 更新城市时变需求和已解决需求
                    for k = 1:length(TSProute)
                        city(TSProute(k)).VaryDemand = city(TSProute(k)).VaryDemand + pretime*increase*city(TSProute(k)).demand - city(TSProute(k)).ttability*pretime;
                        if city(TSProute(k)).VaryDemand < 1e-10
                            city(TSProute(k)).VaryDemand = 0;
                        end
                        city(TSProute(k)).SaveDemand = city(TSProute(k)).SaveDemand + pretime*increase*city(TSProute(k)).demand;
                    end
                    
                    % 发生事件智能体和城市状态更新
                    for j = 1:length(a)
                        if ~(pretime == 0 && robot(a(j)).Ce == 1) % 加入到达中断城市时间和正常到达的解决时间
                            robot(a(j)).temtime = [robot(a(j)).temtime Distime];
                        end
                        
                        if robot(a(j)).Ce == 0 % 上一时刻赶路，即将到达城市
                            % 城市状态更新
                            city(robot(a(j)).target).ttability = city(robot(a(j)).target).ttability + robot(a(j)).abil; % 更新总执行能力
                            city(robot(a(j)).target).NR = city(robot(a(j)).target).NR + 1; % 更新城市智能体数目
                            if robot(a(j)).target ~= 1
                                city(robot(a(j)).target).onway = city(robot(a(j)).target).onway - 1;
                            end
                            
                            % 智能体状态更新
                            robot(a(j)).Ce = robot(a(j)).target; % 更新智能体工作状态
                            robot(a(j)).remainway = 0; % 更新剩余路程
                            robot(a(j)).DisTraveled = robot(a(j)).DisTraveled + Distance(robot(a(j)).delete(end),robot(a(j)).target); % 总里程数更新
                        else % 上一时刻完成任务，即将赶路
                            % 城市状态更新
                            city(robot(a(j)).Ce).finishtime = Distime; % 更新任务完成时间
                            city(robot(a(j)).target).pathlist = [];
                            city(robot(a(j)).Ce).ttability = 0; % 更新总执行能力
                            city(robot(a(j)).Ce).NR = 0; % 更新城市智能体数目
                            
                            % 充电加燃料后智能体状态更新
                            if robot(a(j)).Ce == 1
                                robot(a(j)).DisTraveled = 0;
                                robot(a(j)).delivery = 0;
                            end
                            
                            if robot(a(j)).target ~= 1 % 防止中间出现两个1
                                robot(a(j)).delete = [robot(a(j)).delete, robot(a(j)).target];
                            end
                            
                            % TSP路径中排除已安排的城市
                            TSProute = setdiff(TSProute, robot(a(j)).delete);
                            robot(a(j)).VRProute = [robot(a(j)).VRProute, robot(a(j)).delete(end)];
                            
                            if isempty(TSProute) % 完成所有任务
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
                                incre_v = city(TSProute(k)).demand*increase/ability;
                                sol_v = city(TSProute(k)).NR + city(TSProute(k)).onway;
                                
                                if TSProute(k) <= CityCount
                                    count = count + 1;
                                    SaveTSP = [SaveTSP, TSProute(k)];
                                end
                                
                                if Distance(robot(a(j)).delete(end),TSProute(k))/v > city(TSProute(k)).VaryDemand/(city(TSProute(k)).ttability-city(TSProute(k)).demand*increase) && city(TSProute(k)).ttability > city(TSProute(k)).demand*increase
                                    Eta_d(robot(a(j)).delete(end),TSProute(k)) = 0;
                                elseif sol_v/incre_v > omega
                                    Eta_d(robot(a(j)).delete(end),TSProute(k)) = 1 - sol_v/(incre_v+sol_v);
                                elseif sol_v/incre_v <= 1 && Fai <= cF
                                    Eta_d(robot(a(j)).delete(end),TSProute(k)) = 1 + sol_v/(incre_v+sol_v);
                                elseif sol_v/incre_v <= 1+(omega-1)*Fai/(Fai+cF) && Fai > cF
                                    Eta_d(robot(a(j)).delete(end),TSProute(k)) = 1 + sol_v/(incre_v+sol_v);
                                else
                                    Eta_d(robot(a(j)).delete(end),TSProute(k)) = 1;
                                end
                            end
                            
                            for k1 = 1:length(TSProute)
                                robot(a(j)).P(k1) = robot(a(j)).Tau(robot(a(j)).delete(end),TSProute(k1))*Eta_d(robot(a(j)).delete(end),TSProute(k1))*city(TSProute(k1)).demand*Eta_s(robot(a(j)).delete(end),TSProute(k1))^ar;
                            end
                            
                            % 根据概率矩阵找目标
                            if all(robot(a(j)).P == 0)
                                robot(a(j)).target = 1; % 回仓库
                            else
                                robot(a(j)).P = robot(a(j)).P/sum(robot(a(j)).P);
                                Pc = cumsum(robot(a(j)).P); % 累加概率
                                TargetIndex = find(Pc >= rand);
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
                                robot(a(j)).remainway = Distance(robot(a(j)).Ce,1);
                                robot(a(j)).delete = 1;
                            elseif robot(a(j)).DisTraveled + Distance(robot(a(j)).delete(end),robot(a(j)).target) + Distance(robot(a(j)).target,1) > Travelcon
                                avaicount = 0;
                                if availength > 0
                                    for k = 1:availength
                                        if robot(a(j)).DisTraveled + Distance(robot(a(j)).delete(end),TSProute(availength)) + Distance(TSProute(availength),1) <= Travelcon && city(TSProute(availength)).VaryDemand/(city(TSProute(availength)).ttability-city(TSProute(availength)).demand*increase) > Distance(robot(a(j)).delete(end),TSProute(availength))/v
                                            Record = robot(a(j)).Tau(robot(a(j)).delete(end),TSProute(k))*Eta_d(robot(a(j)).delete(end),TSProute(k))*city(TSProute(k)).demand*Eta_s(robot(a(j)).delete(end),TSProute(k))^ar;
                                            if Record > Save
                                                Save = Record;
                                                robot(a(j)).target = TSProute(availength);
                                                robot(a(j)).remainway = Distance(robot(a(j)).Ce,robot(a(j)).target);
                                            end
                                            avaicount = avaicount + 1;
                                        end
                                    end
                                end
                                
                                if availength == 0 || avaicount == 0
                                    robot(a(j)).target = 1;
                                    robot(a(j)).departure = [];
                                    robot(a(j)).remainway = Distance(robot(a(j)).Ce,1);
                                    robot(a(j)).delete = 1;
                                end
                                
                                if robot(a(j)).target ~= 1
                                    city(robot(a(j)).target).onway = city(robot(a(j)).target).onway + 1;
                                end
                            else
                                robot(a(j)).remainway = Distance(robot(a(j)).Ce,robot(a(j)).target);
                                if robot(a(j)).target ~= 1
                                    city(robot(a(j)).target).onway = city(robot(a(j)).target).onway + 1;
                                end
                            end
                            
                            if ~(all(robot(a(j)).P == 0) && robot(a(j)).Ce == 1) % 排除已返回仓库且无任务需求的机器人
                                robot(a(j)).Ce = 0; % 更新智能体工作状态
                            end
                        end
                    end
                end
                
                %% 路径处理及记录最优路径
                for a = 1:M
                    fillwithones = linspace(1,1,CityNum*2+1-length(robot(a).VRProute));
                    fillwithtimes = linspace(Inf,Inf,CityNum*4+2-length(robot(a).temtime));
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
                [~,min_index] = min(ttltime(1:i));
                
                if i == min_index
                    for a = 1:M
                        robot(a).besttime = robot(a).temtime;
                    end
                end
            end
            
            %% 存储历史最优信息
            if Iter == 1
                [min_Length,min_index] = min(ttltime);
                MinTime(Iter) = min_Length;
                best_Population = Population;
                best_ttltime = ttltime;
                best_index = min_index;
                
                for a = 1:M
                    bestind(a,:) = Population(a,:,min_index);
                    besttime(a,:) = robot(a).besttime;
                end
            else
                [min_Length,min_index] = min(ttltime);
                MinTime(Iter) = min(MinTime(Iter-1), min_Length);
                
                if min_Length == MinTime(Iter)
                    for a = 1:M
                        bestind(a,:) = Population(a,:,min_index);
                        besttime(a,:) = robot(a).besttime;
                    end
                    best_Population = Population;
                    best_ttltime = ttltime;
                    best_index = min_index;
                end
            end
            
            %% 更新算法参数
            for a = 1:M
                robot(a).Delta_Tau = zeros(CityNum*3);
                
                for j = 1:2*CityNum
                    robot(a).Delta_Tau(Population(a,j,min_index),Population(a,j+1,min_index)) = robot(a).Delta_Tau(Population(a,j,min_index),Population(a,j+1,min_index)) + ((mintime/ttltime(min_index)))*save_rate*exp(-Iter*Taumin);
                    
                    if Iter ~= 1
                        robot(a).Delta_Tau(best_Population(a,j,best_index),best_Population(a,j+1,best_index)) = robot(a).Delta_Tau(best_Population(a,j,best_index),best_Population(a,j+1,best_index)) + best_rate;
                    end
                end
                
                robot(a).Tau = (robot(a).Tau + robot(a).Delta_Tau);
            end
            
            for a = 1:M
                robot(a).Tau = robot(a).Tau/(1+save_rate);
            end
            
            for a = 1:M
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
            result_record(Init_k,cycle_num,Iter) = MinTime(Iter);
            time_record(Init_k,cycle_num,Iter) = toc - Init_t;
            
            %% 更新迭代次数
            Iter = Iter + 1;
        end
        
        tem_toc = toc;
        
        %% 找出历史最短距离和对应路径
        mindisever = MinTime(Iter);
        disp('-------------------------------------------------------------')
        toc
        fprintf('Total time= %s \n', num2str(mindisever))
        TextOutput_2(Distance, bestind)
        disp('-------------------------------------------------------------')
        pause(0.2);
        
        cyclebest(Init_k,cycle_num) = mindisever;
        Iter_record(Init_k,cycle_num) = Iter;
    end
end