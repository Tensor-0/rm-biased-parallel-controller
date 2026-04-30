%% =========================================================================
% Balance_Luntuimatlab.m — 偏置并联轮腿机器人 LQR 增益计算
% =========================================================================
%
% 🐣 这个函数是"控制算法的数学核心"——给定一根虚拟腿的长度,
%    用牛顿力学建立三体动力学模型, 用符号推导求雅可比矩阵,
%    用 MATLAB lqr() 算出最优增益矩阵 K[2×6]。
%
%    v3.1 改进:
%     - 机械参数从 robot_params.m 读取 (不再硬编码)
%     - 添加详细注释
%     - 支持连续/离散两种 LQR 求解方法
%
% 输入: leg_length — 虚拟腿长度 (米), 范围 [0.10, 0.40]
% 输出: k         — 2×6 LQR 增益矩阵
%
% 用法: k = Balance_Luntuimatlab(0.18);
%
% 模型说明:
%   ┌── 机体(M) 绕髋关节转动 (φ)
%   ├── 虚拟腿(mp) 绕轮轴转动 (θ)
%   └── 轮子(mw) 水平移动 (x)
%
%   控制输入: T(轮子力矩), Tp(髋关节扭矩)
%   状态向量: [θ, dθ, x, dx, φ, dφ]
% =========================================================================

function k = Balance_Luntuimatlab(leg_length)
    %% ---- 0. 加载机械参数 ----
    run('robot_params.m');   % 读取共享参数文件

    %% ---- 1. 声明符号变量 ----
    %   t-dependent 函数 (位移/角度随时间变化)
    syms x0(t) T R Iw mw M L Lm theta0(t) l phi0(t) mp g Tp Ip Im
    %   简写符号 (用于替换微分表达式)
    syms d2_theta d2_phi d2_x d_theta d_x d_phi theta x phi

    %% ---- 2. 代入数值参数 (机械尺寸) ----
    %   🐣 这些是机器人的"身份证参数"——每台车可能不同
    R_val   = R1;                         % 驱动轮半径
    L_val   = leg_length / 2;            % 摆杆重心到轮轴距离 (虚拟腿一半)
    LM_val  = leg_length / 2;            % 摆杆重心到转轴距离 (同上)
    l_val   = l1;                         % 机体质心到转轴
    mw_val  = mw1;                        % 轮子质量
    mp_val  = mp1;                        % 摆杆质量
    M_val   = M1;                         % 机体质量
    Iw_val  = mw_val * R_val^2;          % 轮子转动惯量 (圆盘 I=mr²)
    %   🐣 摆杆转动惯量: I=mL² (均匀杆绕一端,近似)
    Ip_val  = mp_val * ((L_val + LM_val)^2);
    %   🐣 机体转动惯量: 0.109m 是机体的等效回转半径
    IM_val  = M_val * (0.109^2);
    g_val_s = g_val;                      % 重力加速度

    %% ---- 3. 牛顿力学方程 ----
    %   ========================================
    %   符号推导三组运动方程,每个方程表达一个物体的受力平衡
    %   N,Nm = 水平反力, P,Pm = 垂直反力
    %   ========================================

    % 3.1 水平方向受力分析
    %   Nm: 机体给摆杆的水平反力 (牛顿第二定律: F=ma 在摆杆质心的水平投影)
    Nm = M * diff(x0 + (L + Lm)*sin(theta0) - l*sin(phi0), t, 2);
    %   N: 驱动轮给摆杆的水平反力 (摆杆自身也有惯性力)
    N  = Nm + mp * diff(x0 + L*sin(theta0), t, 2);

    % 3.2 垂直方向受力分析
    %   Pm: 机体给摆杆的垂直反力 (包括重力)
    Pm = M*g_val_s + M * diff((L+Lm)*cos(theta0) + l*cos(phi0), t, 2);
    %   P: 驱动轮给摆杆的垂直反力
    P  = Pm + mp*g_val_s + mp*diff(L*cos(theta0), t, 2);

    % 3.3 三个运动方程 (牛顿第二定律 + 转动方程)
    %   eqn1: 轮子水平运动
    %     🐣 F=ma 的转动版: 力矩 ÷ 等效质量 = 加速度
    eqn1 = diff(x0, t, 2) == (T - N*R_val) ...        % 合力矩
                             / (Iw_val/R_val + mw_val*R_val); % 等效转动惯量

    %   eqn2: 摆杆绕轮轴转动
    %     🐣 Iα = Στ: 转动惯量×角加速度 = 所有力矩的代数和
    eqn2 = Ip_val * diff(theta0, t, 2) ...
         == (P*L_val + Pm*LM_val)*sin(theta0) ...       % 重力产生的转动力矩
          - (N*L_val + Nm*LM_val)*cos(theta0) ...        % 水平反力产生的力矩
          - T + Tp;                                      % 控制输入力矩

    %   eqn3: 机体绕髋关节转动
    %     🐣 机身点头/后仰的动力学
    eqn3 = IM_val * diff(phi0, t, 2) ...
         == Tp + Nm*l_val*cos(phi0) + Pm*l_val*sin(phi0);

    %% ---- 4. 符号替换 (把 diff 表达式换成简写符号) ----
    %   🐣 MATLAB 符号推导中的 diff(x0,t,2) 不能直接用于 jacobian,
    %       所以先把它们换成普通的符号变量 d2_x, d2_theta 等
    sub_list = {diff(theta0,t,2), d2_theta;   diff(x0,t,2), d2_x;
                diff(phi0,t,2),   d2_phi;     diff(theta0,t), d_theta;
                diff(x0,t),       d_x;         diff(phi0,t),   d_phi;
                theta0,           theta;       x0,             x;
                phi0,             phi};
    eqn10 = subs(eqn1, sub_list(:,1), sub_list(:,2));
    eqn20 = subs(eqn2, sub_list(:,1), sub_list(:,2));
    eqn30 = subs(eqn3, sub_list(:,1), sub_list(:,2));

    %% ---- 5. 求解加速度表达式 ----
    %   联立三个方程,解出 d²θ, d²x, d²φ 的显式表达
    [d2_theta_expr, d2_x_expr, d2_phi_expr] = ...
        solve(eqn10, eqn20, eqn30, d2_theta, d2_x, d2_phi);

    %% ---- 6. 线性化: 求雅可比矩阵 → 状态空间 A,B ----
    %   状态向量: [θ, dθ, x, dx, φ, dφ]
    state_var    = [theta, d_theta, x, d_x, phi, d_phi];
    state_deriv  = [d_theta, d2_theta_expr, d_x, d2_x_expr, d_phi, d2_phi_expr];

    %   A = ∂f/∂x | 平衡点 (所有状态=0, 输入=0)
    A_sym = jacobian(state_deriv, state_var);
    A_sym = subs(A_sym, [theta, d_theta, d_x, phi, d_phi, T, Tp], ...
                         [0,     0,       0,   0,   0,     0, 0]);

    %   代入数值
    param_list = {R_val, L_val, LM_val, l_val, mw_val, mp_val, M_val, ...
                  Iw_val, Ip_val, IM_val, g_val_s};
    A = double(subs(A_sym, ...
        {R, L, Lm, l, mw, mp, M, Iw, Ip, Im, g}, param_list));

    %   B = ∂f/∂u | 平衡点
    B_sym = jacobian(state_deriv, [T, Tp]);
    B_sym = subs(B_sym, [theta, d_theta, d_x, phi, d_phi, T, Tp], ...
                         [0,     0,       0,   0,   0,     0, 0]);
    B = double(subs(B_sym, ...
        {R, L, Lm, l, mw, mp, M, Iw, Ip, Im, g}, param_list));

    %% ---- 7. LQR 求解 ----
    switch lqr_method
        case 'discrete'
            % 离散 LQR (旧版本)
            sys = ss(A, B, [], []);
            sys_d = c2d(sys, Ts);
            k = dlqr(sys_d.A, sys_d.B, Q_val, R_val);
        otherwise
            % 连续 LQR (当前版本)
            k = lqr(A, B, Q_val, R_val);
    end
end
