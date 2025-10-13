function grps = assignGrpsWithCDF(M, probs,aggrenum)
    % 确定每组可能的人数范围
    maxGrpSize = min(ceil(M / aggrenum), length(probs));
    
    % 计算累积分布函数（CDF）
    cdf = cumsum(probs(1:maxGrpSize)) / sum(probs(1:maxGrpSize));
    
    % 初始化分组结果，使用动态数组
    grps = [];
    remPeople = M;
    
    % 当还有剩余人数时，继续分配
    while remPeople > 0
        % 如果剩余人数小于最大组大小，调整CDF
        if remPeople < maxGrpSize
            currCDF = cdf(1:remPeople);
            currCDF(end) = 1; % 确保CDF的最后一个值是1
        else
            currCDF = cdf;
        end
        
        % 生成一个随机数，并在CDF中进行二分查找
        r = rand();
        grpSize = find(currCDF >= r, 1, 'first');
        
        % 更新分组结果和剩余人数
        grps(end + 1) = grpSize;
        remPeople = remPeople - grpSize;
    end
    
    % 输出最终的组数
%     N = length(grps);
%     fprintf('最终分成了%d组。\n', N);
end