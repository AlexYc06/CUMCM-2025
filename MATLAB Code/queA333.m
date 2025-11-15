function queA333()
    num_particles = 30;
    max_iter = 200;
    dim = 8;
    lb = [70, 0, 0, 0, 0, 0, 0, 0];
    ub = [140, 360, 67, 67, 67, 19.17, 19.17, 19.17];
    positions = repmat(lb, num_particles, 1) + rand(num_particles, dim) .* repmat(ub - lb, num_particles, 1);
    velocities = zeros(num_particles, dim);
    pbest_positions = positions;
    pbest_values = zeros(num_particles, 1);
    gbest_value = -inf;
    gbest_position = [];
    for i = 1:num_particles
        fit = objective_function(positions(i, :));
        pbest_values(i) = fit;
        if fit > gbest_value
            gbest_value = fit;
            gbest_position = positions(i, :);
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
            velocities(i, :) = w * velocities(i, :) + ...
                c1 * r1 .* (pbest_positions(i, :) - positions(i, :)) + ...
                c2 * r2 .* (gbest_position - positions(i, :));
            positions(i, :) = positions(i, :) + velocities(i, :);
            positions(i, :) = min(max(positions(i, :), lb), ub);
            fit = objective_function(positions(i, :));
            if fit > pbest_values(i)
                pbest_values(i) = fit;
                pbest_positions(i, :) = positions(i, :);
                if fit > gbest_value
                    gbest_value = fit;
                    gbest_position = positions(i, :);
                end
            end
        end
        fprintf('第%d次:迭代结果为：%.2f\n', iter, gbest_value);
    end
    v = gbest_position(1);
    theta = gbest_position(2);
    t_drop1 = gbest_position(3);
    t_drop2 = gbest_position(4);
    t_drop3 = gbest_position(5);
    t_delay1 = gbest_position(6);
    t_delay2 = gbest_position(7);
    t_delay3 = gbest_position(8);
    drone_start = [17800, 0, 1800];
    dir_vector = [cosd(theta), sind(theta), 0];
    drop_point1 = drone_start + v * dir_vector * t_drop1;
    bomb_point1 = drop_point1 + v * dir_vector * t_delay1 - [0, 0, 0.5*9.8*t_delay1^2];
    drop_point2 = drone_start + v * dir_vector * t_drop2;
    bomb_point2 = drop_point2 + v * dir_vector * t_delay2 - [0, 0, 0.5*9.8*t_delay2^2];
    drop_point3 = drone_start + v * dir_vector * t_drop3;
    bomb_point3 = drop_point3 + v * dir_vector * t_delay3 - [0, 0, 0.5*9.8*t_delay3^2];
    fprintf('结果为:\n');
    fprintf('无人机速度: %.2f m/s\n', v);
    fprintf('无人机角度: %.2f deg\n', theta);
    fprintf('第一枚投弹时间: %.2f s\n', t_drop1);
    fprintf('第二枚投弹时间: %.2f s\n', t_drop2);
    fprintf('第二枚投弹时间: %.2f s\n', t_drop3);
    fprintf('第一枚延迟时间: %.2f s\n', t_delay1);
    fprintf('第二枚延迟时间: %.2f s\n', t_delay2);
    fprintf('第三枚延迟时间: %.2f s\n', t_delay3);
    fprintf('第一枚投放点: (%.2f, %.2f, %.2f)\n', drop_point1);
    fprintf('第一枚爆炸点: (%.2f, %.2f, %.2f)\n', bomb_point1);
    fprintf('第二枚投放点: (%.2f, %.2f, %.2f)\n', drop_point2);
    fprintf('第二枚爆炸点: (%.2f, %.2f, %.2f)\n', bomb_point2);
    fprintf('第三枚投放点: (%.2f, %.2f, %.2f)\n', drop_point3);
    fprintf('第三枚爆炸点: (%.2f, %.2f, %.2f)\n', bomb_point3);
    fprintf('总遮蔽时长: %.2f s\n', gbest_value);
end

function total_time = objective_function(X)
    v = X(1);
    theta = X(2);
    t_drop1 = X(3);
    t_drop2 = X(4);
    t_drop3 = X(5);
    t_delay1 = X(6);
    t_delay2 = X(7);
    t_delay3 = X(8);
    if t_drop2 < t_drop1 + 1 || t_drop3 < t_drop2 + 1
        total_time = 0;
        return;
    end
    if t_delay1 > 19.17 || t_drop1 + t_delay1 > 67 || ...
       t_delay2 > 19.17 || t_drop2 + t_delay2 > 67 || ...
       t_delay3 > 19.17 || t_drop3 + t_delay3 > 67
        total_time = 0;
        return;
    end
    drone_start = [17800, 0, 1800];
    dir_vector = [cosd(theta), sind(theta), 0];
    bomb_point1 = drone_start + v * dir_vector * (t_drop1 + t_delay1) - [0, 0, 0.5*9.8*t_delay1^2];
    bomb_point2 = drone_start + v * dir_vector * (t_drop2 + t_delay2) - [0, 0, 0.5*9.8*t_delay2^2];
    bomb_point3 = drone_start + v * dir_vector * (t_drop3 + t_delay3) - [0, 0, 0.5*9.8*t_delay3^2];
    t_explode1 = t_drop1 + t_delay1;
    t_explode2 = t_drop2 + t_delay2;
    t_explode3 = t_drop3 + t_delay3;
    missile_start = [20000, 0, 2000];
    missile_speed = 300;
    missile_direction = -missile_start / norm(missile_start);
    missile_speed_vector = missile_speed * missile_direction;
    points = Points(16, 11);
    t_start = min([t_explode1, t_explode2, t_explode3]);
    t_end = max([t_explode1+20, t_explode2+20, t_explode3+20]);
    dt = 0.1;
    total_time = 0;
    for t = t_start:dt:t_end
        missile_pos = missile_start + missile_speed_vector * t;
        occluded = false(size(points, 1), 1);
        if t >= t_explode1 && t <= t_explode1+20
            bomb_pos1 = bomb_point1 - [0, 0, 3*(t - t_explode1)];
            for j = 1:size(points, 1)
                if ~occluded(j)
                    if is_occluded(missile_pos, points(j, :), bomb_pos1)
                        occluded(j) = true;
                    end
                end
            end
        end
        if t >= t_explode2 && t <= t_explode2+20
            bomb_pos2 = bomb_point2 - [0, 0, 3*(t - t_explode2)];
            for j = 1:size(points, 1)
                if ~occluded(j)
                    if is_occluded(missile_pos, points(j, :), bomb_pos2)
                        occluded(j) = true;
                    end
                end
            end
        end
        if t >= t_explode3 && t <= t_explode3+20
            bomb_pos3 = bomb_point3 - [0, 0, 3*(t - t_explode3)];
            for j = 1:size(points, 1)
                if ~occluded(j)
                    if is_occluded(missile_pos, points(j, :), bomb_pos3)
                        occluded(j) = true;
                    end
                end
            end
        end
        if all(occluded)
            total_time = total_time + dt;
        end
    end
end

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

function points = Points(n, nh)
    center = [0 200 0];
    r = 7;
    h = 10;
    points = zeros(n * nh, 3);
    ind = 1;
    theta = linspace(0, 2 * pi, n);
    z_levels = linspace(center(3), h, nh);
    for j = 1 : nh
        z = z_levels(j);
        for i = 1 : n
            x = center(1) + r * cos(theta(i));
            y = center(2) + r * sin(theta(i));
            points(ind, :) = [x, y, z];
            ind = ind + 1;
        end
    end
end