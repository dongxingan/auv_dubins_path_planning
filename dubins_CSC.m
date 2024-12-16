%% DUBINS PATH规划器
% 功能：基于Dubins曲线的AUV路径规划算法实现
% 作者：董星犴，游子昂
% 版本：1.1
% 日期：20241213
% 主要功能：
%   1. 生成AUV的CSC类型Dubins路径
%   2. 实现多段路径的平滑连接
%   3. 提供路径可视化
% 输出：在工作区生成completePath变量，包含完整路径的位姿数据

clear 
close all
clc

%% 初始化参数设置
% AUV运动学约束参数
minTurningRadius = 3;    % AUV最小转弯半径(m)，决定转弯曲率

% 路径规划参数
numPoints = 16;          % 主路径的规划节点数量
straightDistance = 70;   % 直线段长度(m)，决定探测区域大小
pathInterval = 10;       % 平行航线间距(m)，与探测设备性能相关
additionalPoints = 16;   % 转向段的规划节点数量

% 生成初始路径关键点位姿(x,y,theta)
% pose矩阵每行包含：[x坐标(m), y坐标(m), 航向角(rad)]
pose = pose_calculate(straightDistance, pathInterval, numPoints, additionalPoints);

% 预分配存储空间
numTotalSegments = numPoints + additionalPoints - 1;
pathData = struct('poses', cell(1, numTotalSegments), ...
                 'segment', cell(1, numTotalSegments));

%% 生成主航线路径
h = waitbar(0, '路径规划计算中...');

% 计算主航线段
for i = 1:numPoints-1
    dubinsConn = dubinsConnection('MinTurningRadius', minTurningRadius);
    [pathSegments, ~] = connect(dubinsConn, pose(i,:), pose(i+1,:));
    pathLength = pathSegments{1}.Length;
    pathData(i).poses = interpolate(pathSegments{1}, 0:0.2:pathLength);
    pathData(i).segment = pathSegments{1};
    waitbar(i/(numTotalSegments), h);
end

%% 计算转向段路径
dubinsConn = dubinsConnection('MinTurningRadius', minTurningRadius);
[pathSegments, ~] = connect(dubinsConn, pose(numPoints,:), pose(numPoints+1,:));
pathLength = pathSegments{1}.Length;
pathData(numPoints).poses = interpolate(pathSegments{1}, 0:0.2:pathLength);
pathData(numPoints).segment = pathSegments{1};

%% 生成额外航线路径
for j = 1:additionalPoints-1
    idx = numPoints + j;
    dubinsConn = dubinsConnection('MinTurningRadius', minTurningRadius);
    [pathSegments, ~] = connect(dubinsConn, pose(idx,:), pose(idx+1,:));
    pathLength = pathSegments{1}.Length;
    pathData(idx).poses = interpolate(pathSegments{1}, 0:0.2:pathLength);
    pathData(idx).segment = pathSegments{1};
    waitbar((idx)/(numTotalSegments), h);
end

% 合并所有路径点
completePath = vertcat(pathData.poses);

%% 可视化结果
figure('Name', 'AUV路径规划结果');
% 绘制完整路径
plot(completePath(:,1), completePath(:,2), 'b-', 'LineWidth', 1.5);
hold on;

% 绘制起点和终点标记
plot(completePath(1,1), completePath(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(completePath(end,1), completePath(end,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% 每隔一定点数绘制航向箭头
arrowInterval = 50;  % 每50个点绘制一个航向箭头
quiver(completePath(1:arrowInterval:end,1), ...
       completePath(1:arrowInterval:end,2), ...
       cos(completePath(1:arrowInterval:end,3)), ...
       sin(completePath(1:arrowInterval:end,3)), ...
       0.5, 'r');

% 设置图形属性
grid on;
xlabel('x(m)'), ylabel('y(m)');
axis equal;
title('AUV Dubins路径规划结果');
legend('规划路径', '起点', '终点', '航向', 'Location', 'best');

% 关闭进度条
close(h);

% 输出计算结果统计
fprintf('路径规划完成:\n');
fprintf('总路径段数: %d\n', numTotalSegments);
fprintf('总航点数: %d\n', size(completePath, 1));

%% 导出路径数据到CSV文件（可选）
% 询问用户是否需要保存数据
saveChoice = input('是否需要将路径数据保存为CSV文件？(y/n): ', 's');

if strcmpi(saveChoice, 'y')
    % 使用 datetime 生成当前时间戳
    currentTime = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
    defaultFileName = sprintf('AUV_Dubins_path_%s.csv', char(currentTime));
    
    % 打开保存文件对话框
    [fileName, pathName] = uiputfile({'*.csv', 'CSV文件 (*.csv)'}, ...
        '保存路径数据', defaultFileName);
    
    % 如果用户没有取消保存
    if fileName ~= 0
        % 准备保存的数据
        header = {'X(m)', 'Y(m)', 'Heading(rad)', 'Heading(deg)'};
        saveData = [completePath, completePath(:,3)*180/pi]; % 添加角度列
        
        % 创建完整的文件路径
        fullFilePath = fullfile(pathName, fileName);
        
        % 将数据写入CSV文件
        fid = fopen(fullFilePath, 'w');
        % 写入表头
        fprintf(fid, '%s,%s,%s,%s\n', header{:});
        % 写入数据
        for i = 1:size(saveData, 1)
            fprintf(fid, '%.4f,%.4f,%.4f,%.4f\n', saveData(i,:));
        end
        fclose(fid);
        
        % 显示成功消息
        fprintf('数据已成功保存至：%s\n', fullFilePath);
    end
end

fprintf('数据导出完成\n');



