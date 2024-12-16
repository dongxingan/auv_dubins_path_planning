function [completePath, pathData] = DubinsPathPlanning(minTurningRadius, straightDistance, pathInterval, numPoints, additionalPoints)
%DUBINSPATHPLANNING AUV Dubins路径规划函数
%   基于Dubins曲线的AUV路径规划算法
%
% 输入参数:
%   minTurningRadius  - AUV最小转弯半径(m)
%   straightDistance  - 直线段长度(m)
%   pathInterval     - 平行路径间距(m)
%   numPoints        - 主路径节点数量
%   additionalPoints - 转向段节点数量
%
% 输出参数:
%   completePath     - 完整路径点位姿矩阵 [x, y, heading]
%   pathData         - 路径段数据结构体

    %% 第一步：生成所有关键位姿点
    % 预分配位姿矩阵
    pose = zeros(numPoints + additionalPoints, 3);
    
    % 生成主航线航点
    for i = 1:numPoints
        baseX = 2 * floor((i-1)/4) * pathInterval;  % 计算基准X坐标
        
        % 根据位置确定航点类型和对应的位姿
        if mod(i,4) == 1
            pose(i,:) = [baseX, 0, pi/2];          % 向上航行起点
        elseif mod(i,4) == 2
            pose(i,:) = [baseX, straightDistance, pi/2];  % 向上航行终点
        elseif mod(i,4) == 3
            pose(i,:) = [baseX + pathInterval, straightDistance, -pi/2];  % 向下航行起点
        elseif mod(i,4) == 0
            pose(i,:) = [baseX + pathInterval, 0, -pi/2];  % 向下航行终点
        end
    end
    
    % 生成转向段航点
    for j = numPoints+1 : numPoints+additionalPoints
        baseY = 2*floor(j/4)*pathInterval - numPoints*pathInterval/2;
        lastMainX = 2*floor(numPoints/4)*pathInterval;
        
        if mod(j,4) == 1
            pose(j,:) = [lastMainX-pathInterval, baseY, pi];  % 左转起点
        elseif mod(j,4) == 2
            pose(j,:) = [0, baseY, pi];  % 左转终点
        elseif mod(j,4) == 3
            pose(j,:) = [0, baseY+pathInterval, 0];  % 右转起点
        elseif mod(j,4) == 0
            pose(j,:) = [lastMainX-pathInterval, baseY-pathInterval, 0];  % 右转终点
        end
    end

    %% 第二步：生成Dubins路径
    % 预分配存储空间
    numTotalSegments = numPoints + additionalPoints - 1;
    pathData = struct('poses', cell(1, numTotalSegments), ...
                     'segment', cell(1, numTotalSegments));

    % 计算主航线段
    for i = 1:numPoints-1
        dubinsConn = dubinsConnection('MinTurningRadius', minTurningRadius);
        [pathSegments, ~] = connect(dubinsConn, pose(i,:), pose(i+1,:));
        pathLength = pathSegments{1}.Length;
        pathData(i).poses = interpolate(pathSegments{1}, 0:0.2:pathLength);
        pathData(i).segment = pathSegments{1};
    end

    % 计算转向段路径
    dubinsConn = dubinsConnection('MinTurningRadius', minTurningRadius);
    [pathSegments, ~] = connect(dubinsConn, pose(numPoints,:), pose(numPoints+1,:));
    pathLength = pathSegments{1}.Length;
    pathData(numPoints).poses = interpolate(pathSegments{1}, 0:0.2:pathLength);
    pathData(numPoints).segment = pathSegments{1};

    % 生成额外航线路径
    for j = 1:additionalPoints-1
        idx = numPoints + j;
        dubinsConn = dubinsConnection('MinTurningRadius', minTurningRadius);
        [pathSegments, ~] = connect(dubinsConn, pose(idx,:), pose(idx+1,:));
        pathLength = pathSegments{1}.Length;
        pathData(idx).poses = interpolate(pathSegments{1}, 0:0.2:pathLength);
        pathData(idx).segment = pathSegments{1};
    end

    % 合并所有路径点
    completePath = vertcat(pathData.poses);

    % 输出计算结果统计
    fprintf('路径规划完成:\n');
    fprintf('总路径段数: %d\n', numTotalSegments);
    fprintf('总航点数: %d\n', size(completePath, 1));
end
