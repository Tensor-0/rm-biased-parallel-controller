%% =========================================================================
% get_K.m — LQR 增益调度流水线 (v3.1 优化版)
% =========================================================================
%
% 🐣 这个脚本做 6 件事:
%   1. 读 robot_params.m 获取机械参数和 LQR 权重
%   2. 遍历 31 个腿长采样点, 对每个点调用 Balance_Luntuimatlab
%   3. 收集 12 组 Kij 值 → 用 polyfit 拟合三次多项式
%   4. 画出 12 张拟合质量图 (数据点 vs 拟合线)
%   5. 输出拟合误差统计
%   6. 生成可直接复制到 lqr_controller.c 的 C 代码
%
%   用法: 在 MATLAB 中直接运行 get_K
% =========================================================================

clear; clc; tic;

%% ---- 1. 加载参数 ----
run('robot_params.m');

%% ---- 2. 生成腿长采样序列 ----
leg_samples = leg_min : leg_step : leg_max;
num_samples = length(leg_samples);
fprintf('📐 腿长采样点: %d 个 (%.2f ~ %.2f m, 步长 %.2f m)\n', ...
        num_samples, leg_min, leg_max, leg_step);

%% ---- 3. 逐点计算 LQR 增益 ----
fprintf('🧮 正在计算 LQR 增益矩阵...\n');

j = 1;
for leg = leg_samples
    k = Balance_Luntuimatlab(leg);  % 调用核心函数
    %   提取 K 矩阵的 12 个元素
    k11(j) = k(1,1);  k12(j) = k(1,2);  k13(j) = k(1,3);
    k14(j) = k(1,4);  k15(j) = k(1,5);  k16(j) = k(1,6);
    k21(j) = k(2,1);  k22(j) = k(2,2);  k23(j) = k(2,3);
    k24(j) = k(2,4);  k25(j) = k(2,5);  k26(j) = k(2,6);
    j = j + 1;
    if mod(j, 5) == 0
        fprintf('  已处理 %d/%d 个采样点 (leg=%.2f m, K11=%.3f)\n', ...
                j-1, num_samples, leg, k(1,1));
    end
end
fprintf('  完成! 共 %d 个采样点\n', num_samples);

%% ---- 4. 多项式拟合 ----
fprintf('\n📈 多项式拟合 (阶数=%d)...\n', poly_order);

%   Kij 的行名称 (用于显示和注释)
K_names = {'K11','K12','K13','K14','K15','K16', ...
           'K21','K22','K23','K24','K25','K26'};

%   对每组 kij 做 polyfit
coeffs = cell(1,12);
coeffs{1}  = polyfit(leg_samples, k11, poly_order);
coeffs{2}  = polyfit(leg_samples, k12, poly_order);
coeffs{3}  = polyfit(leg_samples, k13, poly_order);
coeffs{4}  = polyfit(leg_samples, k14, poly_order);
coeffs{5}  = polyfit(leg_samples, k15, poly_order);
coeffs{6}  = polyfit(leg_samples, k16, poly_order);
coeffs{7}  = polyfit(leg_samples, k21, poly_order);
coeffs{8}  = polyfit(leg_samples, k22, poly_order);
coeffs{9}  = polyfit(leg_samples, k23, poly_order);
coeffs{10} = polyfit(leg_samples, k24, poly_order);
coeffs{11} = polyfit(leg_samples, k25, poly_order);
coeffs{12} = polyfit(leg_samples, k26, poly_order);

%% ---- 5. 拟合误差统计 ----
fprintf('\n📊 拟合误差分析 (RMSE = 均方根误差):\n');
fprintf('%-6s  %10s  %10s  %12s\n', 'Kij', '最小值', '最大值', 'RMSE');

k_values = {k11, k12, k13, k14, k15, k16, ...
            k21, k22, k23, k24, k25, k26};
rmse_all = zeros(1,12);

for i = 1:12
    fitted = polyval(coeffs{i}, leg_samples);
    errors = k_values{i} - fitted;
    rmse_all(i) = sqrt(mean(errors.^2));
    fprintf('%-6s  %10.4f  %10.4f  %12.6f\n', ...
            K_names{i}, min(k_values{i}), max(k_values{i}), rmse_all(i));
end

max_rmse = max(rmse_all);
fprintf('最大 RMSE: %.6f %s\n', max_rmse, ...
        iff(max_rmse < 1.0, '✅ 拟合质量良好', '⚠ 拟合误差偏大,考虑提高拟合阶数'));

%% ---- 6. 可视化 (12 子图) ----
figure('Name', 'LQR Gain Scheduling — Polynomial Fit', ...
       'Position', [100, 100, 1200, 800]);

for i = 1:12
    subplot(3, 4, i);
    plot(leg_samples, k_values{i}, 'o', 'MarkerSize', 5);  hold on;
    leg_fine = linspace(leg_min, leg_max, 100);
    plot(leg_fine, polyval(coeffs{i}, leg_fine), 'r-', 'LineWidth', 1.5);
    xlabel('虚拟腿长 L_0 (m)');
    ylabel(K_names{i});
    title(sprintf('%s (RMSE=%.4f)', K_names{i}, rmse_all(i)));
    grid on;
    set(gca, 'GridLineStyle', ':', 'GridColor', 'k', 'GridAlpha', 0.5);
end
sgtitle(sprintf('LQR 增益调度 — 多项式拟合 (腿长 %.2f~%.2f m, 阶数=%d)', ...
        leg_min, leg_max, poly_order));

%% ---- 7. 生成 C 代码 ----
fprintf('\n/* ====== 复制以下代码到 lqr_controller.c ====== */\n');
fprintf('/* 腿长范围: [%.2f, %.2f] m | 拟合阶数: %d | 最大 RMSE: %.6f */\n', ...
        leg_min, leg_max, poly_order, max_rmse);

for i = 1:12
    c = coeffs{i};
    fprintf('float %s[6] = {0, %.6ff, %.6ff, %.6ff, %.6ff};\n', ...
            K_names{i}, c(1), c(2), c(3), c(4));
end

fprintf('/* ====== 复制以上代码到 lqr_controller.c ====== */\n');

%% ---- 8. 生成 Robot_Config.h 注释 (提示同步更新) ----
fprintf('\n💡 提示: 如果修改了 robot_params.m, 请同步更新 Robot_Config.h 中的对应宏:\n');
fprintf('   #define CONF_BODY_MASS      %.2ff\n', M1);
fprintf('   #define CONF_WHEEL_MASS     %.2ff\n', mw1);
fprintf('   #define CONF_LEG_MASS       %.2ff\n', mp1);
fprintf('   #define CONF_WHEEL_RADIUS   %.3ff\n', R1);

toc;
fprintf('\n✅ 全部完成!\n');
