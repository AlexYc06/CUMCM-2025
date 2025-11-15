function main_problem555555()
    % 初始化无人机和导弹位置
    drones = struct('name', {'FY1', 'FY2', 'FY3', 'FY4', 'FY5'}, ...
                   'pos', {[17800, 0, 1800], [12000, 1400, 1400], ...
                          [6000, -3000, 700], [11000, 2000, 1800], ...
                          [13000, -2000, 1300]});
    missiles = struct('name', {'M1', 'M2', 'M3'}, ...
                      'pos', {[20000, 0, 2000], [19000, 600, 2100], ...
                              [18000, -600, 1900]});
    true_target = [0, 200, 0]; % 真目标位置

    % 步骤0: 计算每架无人机对每枚导弹的最大遮蔽时间
    num_particles = 600; % 粒子数量
    max_iter = 200;     % 最大迭代次数
    fprintf('开始计算无人机-导弹时间矩阵...\n');
    [time_matrix, results_precomp] = compute_time_matrix(drones, missiles, true_target, num_particles, max_iter);
    
    % 步骤1: 分配无人机到导弹（每个无人机选择遮蔽时间最长的导弹）
    assignments = assign_drones_to_max_time(drones, missiles, time_matrix);
    
    % 步骤2: 使用预计算的优化结果构建投放策略
    results = cell(1, numel(drones));
    for i = 1:numel(drones)
        drone_name = drones(i).name;
        missile_idx = assignments(i).missile_idx;
        missile_name = missiles(missile_idx).name;
        
        % 从预计算结果中获取最优参数
        gbest_position = results_precomp(i, missile_idx).gbest_position;
        gbest_value = results_precomp(i, missile_idx).gbest_value;
        
        % 解析参数
        v = gbest_position(1);
        theta = gbest_position(2);
        t_drop = gbest_position(3:5);
        t_delay = gbest_position(6:8);
        
        % 存储结果
        results{i} = struct(...
            'drone_name', drone_name, ...
            'missile_name', missile_name, ...
            'v', v, ...
            'theta', theta, ...
            't_drop', t_drop, ...
            't_delay', t_delay, ...
            'individual_time', gbest_value);
    end

    % 步骤3: 计算每枚导弹的总遮蔽时间
    missile_data = struct();
    for m = 1:numel(missiles)
        missile_name = missiles(m).name;
        missile_pos = missiles(m).pos;
        smoke_data = [];
        
        % 收集分配给该导弹的所有烟幕弹
        for r = 1:numel(results)
            if strcmp(results{r}.missile_name, missile_name)
                drone_name = results{r}.drone_name;
                v = results{r}.v;
                theta = results{r}.theta;
                t_drop = results{r}.t_drop;
                t_delay = results{r}.t_delay;
                
                % 找到无人机位置
                drone_idx = find(strcmp({drones.name}, drone_name));
                drone_pos = drones(drone_idx).pos;
                
                % 计算每个烟幕弹的爆炸点和爆炸时间
                for k = 1:3
                    dir_vector = [v * cosd(theta), v * sind(theta), 0];
                    drop_point = drone_pos + dir_vector * t_drop(k);
                    bomb_point = drop_point + dir_vector * t_delay(k) - [0, 0, 0.5*9.8*t_delay(k)^2];
                    t_explode = t_drop(k) + t_delay(k);
                    
                    smoke_data(end+1).bomb_point = bomb_point;
                    smoke_data(end).t_explode = t_explode;
                end
            end
        end
        
        % 计算该导弹的总遮蔽时间
        missile_data(m).name = missile_name;
        missile_data(m).total_time = calculate_missile_time(...
            missile_pos, true_target, smoke_data);
    end

    % 输出最终结果
    fprintf('\n===== 最终结果 =====\n');
    total_occlusion_time = 0;
    for m = 1:numel(missile_data)
        fprintf('导弹 %s 总遮蔽时间: %.2f 秒\n', ...
                missile_data(m).name, missile_data(m).total_time);
        total_occlusion_time = total_occlusion_time + missile_data(m).total_time;
    end
    fprintf('\n三枚导弹总遮蔽时间: %.2f 秒\n', total_occlusion_time);
    
    % 输出各无人机策略
    fprintf('\n===== 无人机投放策略 =====\n');
    for i = 1:numel(results)
        r = results{i};
        fprintf('\n无人机: %s (目标: %s)\n', r.drone_name, r.missile_name);
        fprintf('速度: %.2f m/s, 角度: %.2f°\n', r.v, r.theta);
        for k = 1:3
            fprintf('烟幕弹 %d: 投放时间=%.2fs, 延迟时间=%.2fs\n', ...
                    k, r.t_drop(k), r.t_delay(k));
        end
    end
end

% 计算无人机-导弹时间矩阵（每个无人机对每个导弹优化）
function [time_matrix, results_precomp] = compute_time_matrix(drones, missiles, true_target, num_particles, max_iter)
    nDrones = numel(drones);
    nMissiles = numel(missiles);
    time_matrix = zeros(nDrones, nMissiles);
    results_precomp = struct('gbest_position', cell(nDrones, nMissiles), 'gbest_value', []);
    
    for i = 1:nDrones
        for j = 1:nMissiles
            fprintf('优化无人机 %s 对导弹 %s ...\n', drones(i).name, missiles(j).name);
            [gbest_position, gbest_value] = optimize_drone(...
                drones(i).pos, missiles(j).pos, true_target, num_particles, max_iter);
            
            time_matrix(i, j) = gbest_value;
            results_precomp(i, j).gbest_position = gbest_position;
            results_precomp(i, j).gbest_value = gbest_value;
        end
    end
    
    % 显示时间矩阵
    fprintf('\n无人机-导弹遮蔽时间矩阵:\n');
    drone_names = {drones.name};
    missile_names = {missiles.name};
    fprintf('      \t');
    fprintf('%s\t', missile_names{:});
    fprintf('\n');
    for i = 1:nDrones
        fprintf('%s\t', drone_names{i});
        for j = 1:nMissiles
            fprintf('%.2f\t', time_matrix(i, j));
        end
        fprintf('\n');
    end
end

% 分配无人机到能获得最大遮蔽时间的导弹
function assignments = assign_drones_to_max_time(drones, missiles, time_matrix)
    nDrones = numel(drones);
    nMissiles = numel(missiles);
    assignments = struct('drone_name', {}, 'missile_name', {}, 'missile_idx', {});
    
    % 每个导弹最多分配两个无人机
    missile_count = zeros(1, nMissiles);
    
    % 贪心分配：按时间降序分配
    sorted_pairs = [];
    for i = 1:nDrones
        for j = 1:nMissiles
            sorted_pairs = [sorted_pairs; i, j, time_matrix(i, j)];
        end
    end
    sorted_pairs = sortrows(sorted_pairs, -3); % 按第三列（时间）降序
    
    % 初始化分配结构
    for i = 1:nDrones
        assignments(i).drone_name = drones(i).name;
        assignments(i).missile_name = '';
        assignments(i).missile_idx = 0;
    end
    
    % 第一轮分配：分配最佳匹配
    for k = 1:size(sorted_pairs, 1)
        i = sorted_pairs(k, 1);  % 无人机索引
        j = sorted_pairs(k, 2);  % 导弹索引
        if assignments(i).missile_idx == 0 && missile_count(j) < 2
            missile_count(j) = missile_count(j) + 1;
            assignments(i).missile_name = missiles(j).name;
            assignments(i).missile_idx = j;
        end
    end
    
    % 第二轮分配：处理任何未分配的无人机
    for i = 1:nDrones
        if assignments(i).missile_idx == 0
            % 选择还有空间的导弹
            available_missiles = find(missile_count < 2);
            if ~isempty(available_missiles)
                % 选择时间最长的可用导弹
                best_time = -1;
                best_j = 0;
                for j = available_missiles
                    if time_matrix(i, j) > best_time
                        best_time = time_matrix(i, j);
                        best_j = j;
                    end
                end
                if best_j > 0
                    missile_count(best_j) = missile_count(best_j) + 1;
                    assignments(i).missile_name = missiles(best_j).name;
                    assignments(i).missile_idx = best_j;
                end
            end
        end
    end
    
    % 确保所有无人机都已分配
    for i = 1:nDrones
        if assignments(i).missile_idx == 0
            % 如果还有未分配的无人机，分配到第一个导弹
            assignments(i).missile_name = missiles(1).name;
            assignments(i).missile_idx = 1;
            missile_count(1) = missile_count(1) + 1;
        end
    end
    
    % 显示分配结果
    fprintf('\n无人机分配结果:\n');
    for i = 1:numel(assignments)
        fprintf('%s -> %s\n', assignments(i).drone_name, assignments(i).missile_name);
    end
end

% 优化单架无人机的烟幕弹策略（保持不变）
function [gbest_position, gbest_value] = optimize_drone(drone_start, missile_start, true_target, num_particles, max_iter)
    dim = 8; % 优化变量维度
    lb = [70, 0, 0, 0, 0, 0, 0, 0]; % 下限
    ub = [140, 360, 67, 67, 67, 19.17, 19.17, 19.17]; % 上限
    
    % 初始化粒子群
    positions = repmat(lb, num_particles, 1) + rand(num_particles, dim) .* repmat(ub - lb, num_particles, 1);
    velocities = zeros(num_particles, dim);
    pbest_positions = positions;
    pbest_values = zeros(num_particles, 1);
    gbest_value = -inf;
    gbest_position = [];
    
    % 初始评估
    for i = 1:num_particles
        fit = objective_function(positions(i, :), drone_start, missile_start, true_target);
        pbest_values(i) = fit;
        if fit > gbest_value
            gbest_value = fit;
            gbest_position = positions(i, :);
        end
    end
    
    % PSO主循环
    w_start = 0.9;
    w_end = 0.4;
    c1 = 2;
    c2 = 2;
    
    for iter = 1:max_iter
        w = w_start - (w_start - w_end) * (iter / max_iter);
        for i = 1:num_particles
            % 更新速度和位置
            r1 = rand(1, dim);
            r2 = rand(1, dim);
            velocities(i, :) = w * velocities(i, :) + ...
                c1 * r1 .* (pbest_positions(i, :) - positions(i, :)) + ...
                c2 * r2 .* (gbest_position - positions(i, :));
            
            positions(i, :) = positions(i, :) + velocities(i, :);
            positions(i, :) = min(max(positions(i, :), lb), ub);
            
            % 评估新位置
            fit = objective_function(positions(i, :), drone_start, missile_start, true_target);
            
            % 更新个体和全局最优
            if fit > pbest_values(i)
                pbest_values(i) = fit;
                pbest_positions(i, :) = positions(i, :);
                if fit > gbest_value
                    gbest_value = fit;
                    gbest_position = positions(i, :);
                end
            end
        end
        fprintf('迭代 %d: 当前最优=%.2f\n', iter, gbest_value);
    end
end

% 目标函数：计算单架无人机的遮蔽时间（保持不变）
function total_time = objective_function(X, drone_start, missile_start, true_target)
    % 解析参数
    v = X(1);
    theta = X(2);
    t_drop = X(3:5);
    t_delay = X(6:8);
    
    % 约束检查
    if t_drop(2) < t_drop(1) + 1 || t_drop(3) < t_drop(2) + 1
        total_time = 0;
        return;
    end
    if any(t_delay > 19.17) || any(t_drop + t_delay > 67)
        total_time = 0;
        return;
    end
    
    % 计算烟幕弹爆炸点和爆炸时间
    bomb_points = zeros(3, 3);
    t_explodes = zeros(1, 3);
    dir_vector = [v * cosd(theta), v * sind(theta), 0];
    
    for k = 1:3
        drop_point = drone_start + dir_vector * t_drop(k);
        bomb_points(k, :) = drop_point + dir_vector * t_delay(k) - [0, 0, 0.5*9.8*t_delay(k)^2];
        t_explodes(k) = t_drop(k) + t_delay(k);
    end
    
    % 转换为烟幕数据结构
    smoke_data = struct('bomb_point', {}, 't_explode', {});
    for k = 1:3
        smoke_data(k).bomb_point = bomb_points(k, :);
        smoke_data(k).t_explode = t_explodes(k);
    end
    
    % 计算遮蔽时间
    total_time = calculate_missile_time(missile_start, true_target, smoke_data);
end

% 辅助函数（保持不变）
function points = generate_target_points(center)
    n = 6;   % 圆周方向点数
    nh = 4;  % 高度方向层数
    r = 7;    % 半径
    h = 10;   % 高度
    
    points = zeros(n * nh, 3);
    idx = 1;
    theta = linspace(0, 2*pi, n);
    z_levels = linspace(center(3), center(3) + h, nh);
    
    for j = 1:nh
        z = z_levels(j);
        for i = 1:n
            x = center(1) + r * cos(theta(i));
            y = center(2) + r * sin(theta(i));
            points(idx, :) = [x, y, z];
            idx = idx + 1;
        end
    end
end

% 辅助函数（保持不变）
function flag = is_occluded(A, B, C)
    AB = B - A;
    AC = C - A;
    d_AB = norm(AB);
    if d_AB < 1e-5
        flag = false;
        return;
    end
    t = dot(AC, AB) / (d_AB^2);
    if t < 0
        dist = norm(AC);
    elseif t > 1
        dist = norm(C - B);
    else
        P = A + t * AB;
        dist = norm(C - P);
    end
    flag = (dist <= 10) && (t >= 0) && (t <= 1);
end

% 计算单枚导弹的遮蔽时间（保持不变）
function total_time = calculate_missile_time(missile_start, true_target, smoke_data)
    % 导弹参数
    missile_speed = 300;
    T0 = [0, 0, 0]; % 假目标位置
    missile_dir = (T0 - missile_start) / norm(T0 - missile_start);
    missile_vel = missile_speed * missile_dir;
    
    % 生成真目标取样点
    points = generate_target_points(true_target);
    
    % 确定时间范围
    if isempty(smoke_data)
        total_time = 0;
        return;
    end
    
    t_explodes = [smoke_data.t_explode];
    t_start = min(t_explodes);
    t_end = max(t_explodes) + 20;
    dt = 0.1;
    total_time = 0;
    
    % 时间步进模拟
    for t = t_start:dt:t_end
        missile_pos = missile_start + missile_vel * t;
        occluded = false(size(points, 1), 1);
        
        % 检查每个烟幕弹
        for s = 1:numel(smoke_data)
            if t >= smoke_data(s).t_explode && t <= smoke_data(s).t_explode + 20
                % 计算烟幕弹当前位置（考虑下降）
                fall_dist = 3 * (t - smoke_data(s).t_explode);
                smoke_pos = smoke_data(s).bomb_point - [0, 0, fall_dist];
                
                % 检查每个取样点是否被遮蔽
                for p = 1:size(points, 1)
                    if ~occluded(p)
                        if is_occluded(missile_pos, points(p, :), smoke_pos)
                            occluded(p) = true;
                        end
                    end
                end
            end
        end
        
        % 如果所有点都被遮蔽，累加时间
        if all(occluded)
            total_time = total_time + dt;
        end
    end
end