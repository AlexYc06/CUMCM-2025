function que444()

    missile_start = [20000, 0, 2000];
    missile_target = [0, 0, 0];
    missile_speed = 300;
    s_total = norm(missile_start - missile_target);
    T_total = s_total / missile_speed;
    drone_starts = [17800, 0, 1800;
                   12000, 1400, 1400;
                   6000, -3000, 700];

    lb_common = [70, 0, 0, 0];
    ub_common = [140, 360, 60, 40];
    results = cell(3, 1);
    intervals = [];
    num_particles = 300;
    max_iter = 300;
    lambda = 0.7;
    for drone_idx = 1:3
        fprintf('\n===== 优化无人机 FY%d =====\n', drone_idx);
        max_t_delay = sqrt(2 * drone_starts(drone_idx, 3) / 9.8);
        lb = lb_common;
        ub = ub_common;
        ub(4) = min(ub(4), max_t_delay);
        ub(3) = min(ub(3), T_total - ub(4));
        [best_params, best_interval] = pso_drone_optimization(...
            num_particles, max_iter, lb, ub, T_total, ...
            drone_starts(drone_idx, :), intervals, lambda);
        results{drone_idx} = best_params;
        intervals = [intervals; best_interval];
        fprintf('FY%d 最优参数: 速度=%.2f m/s, 角度=%.2f°, 投放时间=%.2f s, 延迟=%.2f s\n', ...
                drone_idx, best_params(1), best_params(2), best_params(3), best_params(4));
        fprintf('遮蔽时间段: [%.2f, %.2f] s, 持续 %.2f s\n\n', ...
                best_interval(1), best_interval(2), diff(best_interval));
    end
    total_cover_time = compute_union_cover_time(intervals);
    fprintf('\n===== 最终投放策略 =====\n');
    for drone_idx = 1:3
        params = results{drone_idx};
        drone_start = drone_starts(drone_idx, :);
        drop_point = drone_start + [params(1)*cosd(params(2))*params(3), ...
                                   params(1)*sind(params(2))*params(3), ...
                                   0];
        bomb_point = drop_point + [params(1)*cosd(params(2))*params(4), ...
                                  params(1)*sind(params(2))*params(4), ...
                                  -0.5*9.8*params(4)^2];
        fprintf('无人机 FY%d:\n', drone_idx);
        fprintf('  速度: %.2f m/s, 方向角: %.2f°, 投放时间: %.2f s, 延迟时间: %.2f s\n', ...
                params(1), params(2), params(3), params(4));
        fprintf('  投放点: (%.2f, %.2f, %.2f) m\n', drop_point);
        fprintf('  引爆点: (%.2f, %.2f, %.2f) m\n', bomb_point);
        fprintf('  遮蔽时段: [%.2f, %.2f] s (%.2f s)\n', ...
                intervals(drone_idx, 1), intervals(drone_idx, 2), diff(intervals(drone_idx, :)));
    end
    fprintf('\n总有效遮蔽时间: %.2f 秒\n', total_cover_time);
end

function [best_params, best_interval] = pso_drone_optimization(...
        num_particles, max_iter, lb, ub, T_total, drone_start, fixed_intervals, lambda)
    dim = 4;
    positions = repmat(lb, num_particles, 1) + ...
                rand(num_particles, dim) .* repmat(ub - lb, num_particles, 1);
    velocities = zeros(num_particles, dim);
    pbest_positions = positions;
    pbest_values = -inf(num_particles, 1);
    gbest_value = -inf;
    gbest_position = positions(1, :);
    gbest_interval = [0, 0];
    for i = 1:num_particles
        if isempty(fixed_intervals)
            fit = smoke_cover_time(positions(i, :), T_total, drone_start);
        else
            fit = smoke_cover_time_with_penalty(...
                positions(i, :), T_total, drone_start, fixed_intervals, lambda);
        end
        pbest_values(i) = fit;
        if fit > gbest_value
            gbest_value = fit;
            gbest_position = positions(i, :);
            [cover_time, ~, ~, ts, te] = smoke_cover_time(positions(i, :), T_total, drone_start);
            if cover_time > 0
                t_explode = positions(i, 3) + positions(i, 4);
                gbest_interval = [t_explode + ts, t_explode + te];
            end
        end
    end
    w_start = 0.9;
    w_end = 0.4;
    c1 = 2.0;
    c2 = 2.0;
    for iter = 1:max_iter
        w = w_start - (w_start - w_end) * (iter / max_iter);
        for i = 1:num_particles
            r1 = rand(1, dim);
            r2 = rand(1, dim);
            velocities(i, :) = w * velocities(i, :) + ...
                c1 * r1 .* (pbest_positions(i, :) - positions(i, :)) + ...
                c2 * r2 .* (gbest_position - positions(i, :));
            positions(i, :) = positions(i, :) + velocities(i, :);
            positions(i, :) = min(max(positions(i, :), lb), ub);
            if isempty(fixed_intervals)
                fit = smoke_cover_time(positions(i, :), T_total, drone_start);
            else
                fit = smoke_cover_time_with_penalty(...
                    positions(i, :), T_total, drone_start, fixed_intervals, lambda);
            end
            if fit > pbest_values(i)
                pbest_values(i) = fit;
                pbest_positions(i, :) = positions(i, :);
                if fit > gbest_value
                    gbest_value = fit;
                    gbest_position = positions(i, :);
                    [cover_time, ~, ~, ts, te] = smoke_cover_time(positions(i, :), T_total, drone_start);
                    if cover_time > 0
                        t_explode = positions(i, 3) + positions(i, 4);
                        gbest_interval = [t_explode + ts, t_explode + te];
                    end
                end
            end
        end
        fprintf('迭代 %d, 最佳适应度: %.4f\n', iter, gbest_value);
    end
    best_params = gbest_position;
    best_interval = gbest_interval;
end

function [cover_time, drop_point, bomb_point, t_start_rel, t_end_rel] = ...
                smoke_cover_time(X, T_total, drone_start)
    v = X(1);
    theta_deg = X(2);
    t_drop = X(3);
    t_delay = X(4);
    drop_point = drone_start + [v * cosd(theta_deg) * t_drop, ...
                                v * sind(theta_deg) * t_drop, ...
                                0];
    bomb_point = drop_point + [v * cosd(theta_deg) * t_delay, ...
                               v * sind(theta_deg) * t_delay, ...
                               -0.5 * 9.8 * t_delay^2];
    t_explode = t_drop + t_delay;
    if t_explode >= T_total || bomb_point(3) < 0
        cover_time = 0;
        t_start_rel = 0;
        t_end_rel = 0;
        return;
    end
    n_r = 8;
    n_h = 11;
    points = generate_target_points(n_r, n_h);
    missile_dir = (missile_target - missile_start) / norm(missile_target - missile_start);
    missile_pos_explode = missile_start + missile_dir * 300 * t_explode;
    [cover_time, t_start_rel, t_end_rel] = occlusion(...
        bomb_point, missile_pos_explode, points, missile_dir);
end

function fit = smoke_cover_time_with_penalty(...
                X, T_total, drone_start, fixed_intervals, lambda)
    [cover_time, ~, ~, t_start_rel, t_end_rel] = ...
        smoke_cover_time(X, T_total, drone_start);
    if cover_time <= 0
        fit = -1000;
        return;
    end
    t_explode = X(3) + X(4);
    S_i = t_explode + t_start_rel;
    E_i = t_explode + t_end_rel;
    overlap = 0;
    for k = 1:size(fixed_intervals, 1)
        I_k = fixed_intervals(k, :);
        overlap = overlap + max(0, min(E_i, I_k(2)) - max(S_i, I_k(1)));
    end
    fit = cover_time - lambda * overlap;
end

function points = generate_target_points(n_r, n_h)
    center = [0, 200, 0];
    radius = 7;
    height = 10;
    points = zeros(n_r * n_h, 3);
    angles = linspace(0, 2*pi, n_r);
    z_levels = linspace(center(3), center(3) + height, n_h);
    idx = 1;
    for j = 1:n_h
        z = z_levels(j);
        for i = 1:n_r
            x = center(1) + radius * cos(angles(i));
            y = center(2) + radius * sin(angles(i));
            points(idx, :) = [x, y, z];
            idx = idx + 1;
        end
    end
end

function [cover_time, t_start_rel, t_end_rel] = ...
                occlusion(bomb_point, missile_pos_explode, points, missile_dir)
    t_temp = 0.1;
    t_start_rel = -1;
    found_start = false;
    missile_speed_vec = 300 * missile_dir;
    for step = 0:200
        t = step * t_temp;
        if t > 20
            break;
        end
        smoke_pos = bomb_point - [0, 0, 3*t];
        if smoke_pos(3) < 0
            if found_start
                t_end_rel = t;
                cover_time = t_end_rel - t_start_rel;
                return;
            else
                t_start_rel = 0;
                t_end_rel = 0;
                cover_time = 0;
                return;
            end
        end
        missile_pos = missile_pos_explode + missile_speed_vec * t;
        is_occluded = true;
        for j = 1:size(points, 1)
            target_point = points(j, :);
            
            if ~is_point_occluded(missile_pos, target_point, smoke_pos)
                is_occluded = false;
                break;
            end
        end
        if is_occluded
            if ~found_start
                t_start_rel = t;
                found_start = true;
            end
        else
            if found_start
                t_end_rel = t;
                cover_time = t_end_rel - t_start_rel;
                return;
            end
        end
    end
    if found_start
        t_end_rel = 20;
        cover_time = 20 - t_start_rel;
    else
        t_start_rel = 0;
        t_end_rel = 0;
        cover_time = 0;
    end
end

function flag = is_point_occluded(missile_pos, target_point, smoke_pos)
    MT = target_point - missile_pos;
    dist_MT = norm(MT);
    if dist_MT < 1e-6
        flag = false;
        return;
    end
    MS = smoke_pos - missile_pos;
    t = dot(MS, MT) / (dist_MT^2);
    if t < 0
        dist = norm(MS);
    elseif t > 1
        dist = norm(smoke_pos - target_point);
    else
        projection = missile_pos + t * MT;
        dist = norm(smoke_pos - projection);
    end
    flag = (dist <= 10) && (t >= 0) && (t <= 1);
end

function total_cover = compute_union_cover_time(intervals)
    if isempty(intervals)
        total_cover = 0;
        return;
    end
    sorted_intervals = sortrows(intervals, 1);
    total_cover = 0;
    current_start = sorted_intervals(1, 1);
    current_end = sorted_intervals(1, 2);
    for i = 2:size(sorted_intervals, 1)
        if sorted_intervals(i, 1) <= current_end
            current_end = max(current_end, sorted_intervals(i, 2));
        else
            total_cover = total_cover + (current_end - current_start);
            current_start = sorted_intervals(i, 1);
            current_end = sorted_intervals(i, 2);
        end
    end
    total_cover = total_cover + (current_end - current_start);
end

function pos = missile_start()
    pos = [20000, 0, 2000];
end

function pos = missile_target()
    pos = [0, 0, 0];
end