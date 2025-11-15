function queA222
    num_particles = 50;
    max_iter = 100;
    dim = 4;
    lb = [70, 0, 0, 0];
    ub = [140, 360, 67, 19.17];
    s_total = norm([20000, 0, 2000]);
    T_total = s_total / 300;
    positions = repmat(lb, num_particles, 1) +...
        rand(num_particles, dim) .* repmat(ub - lb, num_particles, 1);
    velocities = zeros(num_particles, dim);
    pbest_positions = positions;
    pbest_values = -inf(num_particles, 1);
    gbest_value = -inf;
    gbest_position = [];
    for i = 1:num_particles
        fit = smoke_cover_time(positions(i,:), T_total);
        pbest_values(i) = fit;
        if fit > gbest_value
            gbest_value = fit;
            gbest_position = positions(i,:);
        end
    end
    w_start = 0.9;
    w_end = 0.4;
    c1 = 2;
    c2 = 2;
    for iter = 1:max_iter
        w = w_start - (w_start - w_end) * (iter / max_iter);
        for i = 1:num_particles
            r1 = rand(1, dim);
            r2 = rand(1, dim);
            velocities(i,:) = w * velocities(i,:) + ...
            c1 * r1 .* (pbest_positions(i,:) - positions(i,:)) + ...
                c2 * r2 .* (gbest_position - positions(i,:));
            positions(i,:) = positions(i,:) + velocities(i,:);
            positions(i,:) = min(max(positions(i,:), lb), ub);
            fit = smoke_cover_time(positions(i,:), T_total);
            if fit > pbest_values(i)
                pbest_values(i) = fit;
                pbest_positions(i,:) = positions(i,:);
                if fit > gbest_value
                    gbest_value = fit;
                    gbest_position = positions(i,:);
                end
            end
        end
        fprintf('第%d次迭代最长遮档时间为：%.2f s\n', iter, gbest_value);
    end
    [cover_time, drop_point, bomb_point] = smoke_cover_time(gbest_position, T_total);
    fprintf('\n最终结果:\n');
    fprintf('无人机速度：%.2f m/s\n', gbest_position(1));
    fprintf('无人机方向（与x轴正方向的夹角）： %.2f °\n', gbest_position(2));
    fprintf('投放时间： %.2f s\n', gbest_position(3));
    fprintf('延迟爆炸时间： %.2f s\n', gbest_position(4));
    fprintf('干扰弹投放点： (%.2f, %.2f, %.2f) m\n', drop_point);
    fprintf('干扰弹引爆点： (%.2f, %.2f, %.2f) m\n', bomb_point);
    fprintf('最长遮盖时间： %.2f s\n', cover_time);
    disp(gbest_position);
end

function [cover_time, drop_point, bomb_point] = smoke_cover_time(X, T_total)
    v = X(1);
    theta_deg = X(2);
    t_drop = X(3);
    t_delay = X(4);
    drone_start = [17800, 0, 1800];
    drop_point = drone_start + [v * cosd(theta_deg) * t_drop, ...
                                v * sind(theta_deg) * t_drop, ...
                                0];
    bomb_point = drop_point + [v * cosd(theta_deg) * t_delay, ...
                               v * sind(theta_deg) * t_delay, ...
                               -0.5 * 9.8 * t_delay^2];
    if (t_drop + t_delay) >= T_total || bomb_point(3) < 0
        cover_time = 0;
        return;
    end
    n_circle = 64;
    points = Points(n_circle);
    missile_start = [20000, 0, 2000];
    missile_target = [0, 0, 0];
    t_explode = t_drop + t_delay;
    missile_dir = (missile_target - missile_start) /...
        norm(missile_target - missile_start);
    missile_pos_explode = missile_start + missile_dir * 300 * t_explode;
    cover_time = occlusion(bomb_point, missile_pos_explode, points);
end

function points = Points(n)
    center = [0 200 0];
    r = 7;
    h = 10;
    points = zeros(n * 2, 3);
    ind = 1;
    theta = linspace(0, 2 * pi, n);
    z_levels = linspace(center(3), center(3) + h, 2);
    for j = 1 : 2
        z = z_levels(j);
        for i = 1 : n
            x = center(1) + r * cos(theta(i));
            y = center(2) + r * sin(theta(i));
            points(ind, :) = [x, y, z];
            ind = ind + 1;
        end
    end
end

function t = occlusion(axb, axm, points)
    t_temp = 0.01;
    t_start = -1;
    t_end = -1;
    axm_norm = norm(axm);
    axm_dir = axm / axm_norm;
    axm_step = 300 * axm_dir;
    for i = 0 : (20 / t_temp)
        step_val = i * t_temp;
        axb_temp = axb - step_val * [0 0 3];
        axm_temp = axm - step_val * axm_step;
        v2 = axm_temp - axb_temp;
        d2 = norm(v2);
        d2_sq = d2 * d2;
        v1 = axm_temp - points;
        d1_sq = sum(v1.^2, 2);
        v1_dot_v2 = sum(v1 .* v2, 2);
        cross_norm_sq = d1_sq * d2_sq - v1_dot_v2.^2;
        d1 = sqrt(max(cross_norm_sq, 0)) ./ sqrt(d1_sq);
        v3 = axb_temp - points;
        d3 = sqrt(sum(v3.^2, 2));
        v1_dot_v3 = sum(v1 .* v3, 2);
        cond1 = (d1 > 10);
        cond2 = (d2 > 10) & (v1_dot_v2 < 0);
        cond3 = (d3 > 10) & (v1_dot_v3 < 0);
        if_occlusion = ~any(cond1 | cond2 | cond3);
        if if_occlusion
            if t_start == -1
                t_start = step_val;
            end
        else
            if t_start ~= -1 && t_end == -1
                t_end = step_val;
                t = t_end - t_start;
                return;
            end
        end
    end
    if if_occlusion
        t = 20 - t_start;
    elseif t_start == -1
        t = 0;
    else
        t = 0;
    end
end