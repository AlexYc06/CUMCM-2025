function t_ans = queA111()
    g = 9.8;
    x_fy1 = 17800 - 1.5 * 120;
    x_boom = x_fy1 - 3.6 * 120;
    z_boom = 1800 - 0.5 * g * 3.6 * 3.6;
    x_M1 = 20000 - 5.1 * 300 * 10 / sqrt(101);
    z_M1 = 2000 - 5.1 * 300 / sqrt(101);
    ax_boom = [x_boom 0 z_boom];
    ax_M1 = [x_M1 0 z_M1];
    points = Points(64);
    t_ans = occlusion(ax_boom, ax_M1, points);
end

function points = Points(n)
    center = [0 200 0];
    r = 7;
    h = 10;
    points = zeros(n * 2, 3);
    ind = 1;
    theta = linspace(0, 2 * pi, n);
    z_levels = linspace(center(3), h, 2);
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
        v1_dot_v2 = v1 * v2';
        cross_norm_sq = sum(v1.^2, 2) * d2_sq - v1_dot_v2.^2;
        d1 = sqrt(max(cross_norm_sq, 0)) ./ sqrt(sum(v1.^2, 2));
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
    end
end