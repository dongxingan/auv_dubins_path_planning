function [pose] = pose_calculate(straightDistance, pathInterval, mainPoints, turnPoints)
%POSE_CALCULATE 计算AUV路径规划的关键位姿点
%
% 输入参数:
%   straightDistance - 直线段长度(m)
%   pathInterval    - 平行路径间隔(m)
%   mainPoints      - 主路径点数量
%   turnPoints      - 转向路径点数量
%
% 输出:
%   pose - 包含所有路径点位姿的矩阵 [x, y, heading]
%          x, y: 位置坐标(m)
%          heading: 航向角(rad)，定义：
%                  pi/2: 向上
%                  -pi/2: 向下
%                  0: 向右
%                  pi: 向左

    % 预分配存储空间
    pose = zeros(mainPoints + turnPoints, 3);
    
    %% 第一部分：生成主航线航点
    for i = 1:mainPoints
        baseX = 2 * floor((i-1)/4) * pathInterval;  % 计算基准X坐标
        
        % 根据位置确定航点类型和对应的位姿
        if mod(i,4) == 1
            % 向上航行起点
            pose(i,:) = [baseX, 0, pi/2];
            
        elseif mod(i,4) == 2
            % 向上航行终点
            pose(i,:) = [baseX, straightDistance, pi/2];
            
        elseif mod(i,4) == 3
            % 向下航行起点
            pose(i,:) = [baseX + pathInterval, straightDistance, -pi/2];
            
        elseif mod(i,4) == 0
            % 向下航行终点
            pose(i,:) = [baseX + pathInterval, 0, -pi/2];
        end
    end
    
    %% 第二部分：生成转向段航点
    for j = mainPoints+1 : mainPoints+turnPoints
        % 计算转向段基准坐标
        baseY = 2*floor(j/4)*pathInterval - mainPoints*pathInterval/2;
        lastMainX = 2*floor(mainPoints/4)*pathInterval;
        
        % 根据位置确定转向航点类型和对应的位姿
        if mod(j,4) == 1
            % 左转起点
            pose(j,:) = [lastMainX-pathInterval, baseY, pi];
            
        elseif mod(j,4) == 2
            % 左转终点
            pose(j,:) = [0, baseY, pi];
            
        elseif mod(j,4) == 3
            % 右转起点
            pose(j,:) = [0, baseY+pathInterval, 0];
            
        elseif mod(j,4) == 0
            % 右转终点
            pose(j,:) = [lastMainX-pathInterval, baseY-pathInterval, 0];
        end
    end
end