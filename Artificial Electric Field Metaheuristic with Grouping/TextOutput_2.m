function TextOutput_2(Distance,bestroute)
%% 输出路径函数
%输入：route 路径
%输出：p 路径文本形式

for i=1:length(bestroute(:,1))
    route=bestroute(i,:);
    for j=1:length(route)-1
        if route(j)==route(j+1) %相邻位都为1时
           route(j)=0;  %前一个置零
        end
    end
    route(route==0)=[];  %删去多余零元素 
    
    DisTraveled=0;  % 汽车已经行驶的距离
    subpath='0'; %子路径路线 
   
    for j=2:length(route)
        DisTraveled = DisTraveled+Distance(route(j-1),route(j)); %每两点间距离累加
        subpath=[subpath,' -> ',num2str(route(j)-1)]; %子路径路线输出
    end
    disp('-------------------------------------------------------------')      
    fprintf('Route of Vehichle No.%d: %s  \n',i,subpath)%输出：每辆车 路径       
    fprintf('Distance traveled: %.2f  \n',DisTraveled)%输出：行驶距离     
end

end