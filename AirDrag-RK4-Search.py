import numpy as np
import matplotlib.pyplot as plt

def calculate_range(angle_deg, v0=10, k=0.05, g=9.80, dt=0.001, max_time=10):
    """
    使用四阶龙格-库塔法计算给定角度的射程
    angle_deg: 投掷角度（度）
    v0: 初始速度（m/s）
    k: 空气阻力系数
    g: 重力加速度
    dt: 时间步长
    max_time: 最大模拟时间
    """
    # 转换为弧度
    angle_rad = np.radians(angle_deg)
    
    # 初始条件
    x, y = 0, 0
    vx = v0 * np.cos(angle_rad)
    vy = v0 * np.sin(angle_rad)
    
    # 存储轨迹
    trajectory = [(x, y)]
    
    # 模拟直到落地或超时
    t = 0
    while y >= 0 and t < max_time:
        # 当前速度大小
        v = np.sqrt(vx**2 + vy**2)
        
        # 四阶龙格-库塔法
        # k1
        k1_vx = -k * v * vx
        k1_vy = -g - k * v * vy
        k1_x = vx
        k1_y = vy
        
        # k2 (使用k1的中间值)
        v_mid = np.sqrt((vx + 0.5*dt*k1_vx)**2 + (vy + 0.5*dt*k1_vy)**2)
        k2_vx = -k * v_mid * (vx + 0.5*dt*k1_vx)
        k2_vy = -g - k * v_mid * (vy + 0.5*dt*k1_vy)
        k2_x = vx + 0.5*dt*k1_vx
        k2_y = vy + 0.5*dt*k1_vy
        
        # k3 (使用k2的中间值)
        v_mid = np.sqrt((vx + 0.5*dt*k2_vx)**2 + (vy + 0.5*dt*k2_vy)**2)
        k3_vx = -k * v_mid * (vx + 0.5*dt*k2_vx)
        k3_vy = -g - k * v_mid * (vy + 0.5*dt*k2_vy)
        k3_x = vx + 0.5*dt*k2_vx
        k3_y = vy + 0.5*dt*k2_vy
        
        # k4 (使用k3的最终值)
        v_mid = np.sqrt((vx + dt*k3_vx)**2 + (vy + dt*k3_vy)**2)
        k4_vx = -k * v_mid * (vx + dt*k3_vx)
        k4_vy = -g - k * v_mid * (vy + dt*k3_vy)
        k4_x = vx + dt*k3_vx
        k4_y = vy + dt*k3_vy
        
        # 更新速度和位置
        vx += (dt / 6) * (k1_vx + 2*k2_vx + 2*k3_vx + k4_vx)
        vy += (dt / 6) * (k1_vy + 2*k2_vy + 2*k3_vy + k4_vy)
        x += (dt / 6) * (k1_x + 2*k2_x + 2*k3_x + k4_x)
        y += (dt / 6) * (k1_y + 2*k2_y + 2*k3_y + k4_y)
        
        trajectory.append((x, y))
        t += dt
    
    return x, trajectory

def find_optimal_angle_for_params(v0, k, low=30.0, high=50.0, tol=0.1, max_iter=10):
    """
    针对给定的速度和阻力系数寻找最优角度
    """
    for i in range(max_iter):
        mid1 = low + (high - low) / 3
        mid2 = high - (high - low) / 3
        
        range_mid1, _ = calculate_range(mid1, v0, k)
        range_mid2, _ = calculate_range(mid2, v0, k)
        
        if range_mid1 < range_mid2:
            low = mid1
        else:
            high = mid2
        
        if high - low < tol:
            break
    
    return (low + high) / 2

def ternary_search_optimization(v0=10, k=0.05, low=30.0, high=50.0, tol=0.1, max_iter=200):
    """
    使用三分法寻找最优角度
    tol: 角度容差（度）
    max_iter: 最大迭代次数
    """
    print("使用三分法寻找最优角度（考虑空气阻力）...")
    print(f"初始速度: {v0} m/s, 空气阻力系数: {k}")
    print(f"搜索范围: [{low}°, {high}°], 容差: {tol}°")
    print("-" * 60)
    
    convergence_history = []
    
    for i in range(max_iter):
        # 计算两个内点
        mid1 = low + (high - low) / 3
        mid2 = high - (high - low) / 3
        
        # 计算四个点的射程
        range_low, _ = calculate_range(low, v0, k)
        range_mid1, _ = calculate_range(mid1, v0, k)
        range_mid2, _ = calculate_range(mid2, v0, k)
        range_high, _ = calculate_range(high, v0, k)
        
        # 记录收敛历史
        convergence_history.append({
            'iteration': i + 1,
            'low': low,
            'mid1': mid1,
            'mid2': mid2,
            'high': high,
            'range_low': range_low,
            'range_mid1': range_mid1,
            'range_mid2': range_mid2,
            'range_high': range_high
        })
        
        print(f"迭代 {i+1}: 区间 [{low:.4f}°, {high:.4f}°]")
        print(f"  低点: {low:.4f}° -> 射程 {range_low:.6f}m")
        print(f"  中点1: {mid1:.4f}° -> 射程 {range_mid1:.6f}m")
        print(f"  中点2: {mid2:.4f}° -> 射程 {range_mid2:.6f}m")
        print(f"  高点: {high:.4f}° -> 射程 {range_high:.6f}m")
        
        # 三分法决策
        if range_mid1 < range_mid2:
            # 最大值在[mid1, high]区间
            low = mid1
        else:
            # 最大值在[low, mid2]区间
            high = mid2
        
        # 检查收敛条件
        if high - low < tol:
            print(f"收敛条件满足：区间长度 {high-low:.6f}° < 容差 {tol}°")
            break
    
    # 最终确定最优角度（取区间中点）
    optimal_angle = (low + high) / 2
    max_range, optimal_trajectory = calculate_range(optimal_angle, v0, k)
    
    print("-" * 60)
    print(f"最优角度: {optimal_angle:.6f}°")
    print(f"最大射程: {max_range:.6f} m")
    print(f"迭代次数: {len(convergence_history)}")
    print(f"最终区间: [{low:.6f}°, {high:.6f}°]")
    
    return optimal_angle, max_range, optimal_trajectory, convergence_history

def compare_with_no_air_resistance(v0=10, g=9.80):
    """计算无空气阻力情况下的最优角度和最大射程"""
    # 无空气阻力时最优角度为45度
    optimal_angle_no_air = 45.0
    max_range_no_air = (v0**2 * np.sin(2 * np.radians(optimal_angle_no_air))) / g
    
    print("\n无空气阻力情况:")
    print(f"最优角度: {optimal_angle_no_air}°")
    print(f"最大射程: {max_range_no_air:.6f} m")
    print(f"理论公式: R = v₀²sin(2θ)/g")
    
    return optimal_angle_no_air, max_range_no_air

def plot_results(optimal_angle, max_range, optimal_trajectory, convergence_history):
    """绘制结果"""
    # 绘制最优轨迹
    x_vals = [point[0] for point in optimal_trajectory]
    y_vals = [point[1] for point in optimal_trajectory]
    
    plt.figure(figsize=(15, 10))
    
    # 子图1: 最优轨迹
    plt.subplot(2, 2, 1)
    plt.plot(x_vals, y_vals, 'b-', linewidth=2, label=f'best trajectory ({optimal_angle:.2f}degrees)')
    plt.scatter(max_range, 0, color='red', s=100, marker='o', label=f'ground point ({max_range:.2f}m)')
    
    # 绘制无空气阻力的45度轨迹作为对比
    angle_45_range, angle_45_traj = calculate_range(45, k=0)
    x_45 = [point[0] for point in angle_45_traj]
    y_45 = [point[1] for point in angle_45_traj]
    plt.plot(x_45, y_45, 'g--', linewidth=1, label='45degrees nomarl without air')
    plt.scatter(angle_45_range, 0, color='green', s=80, marker='x', label=f'45degrees ground point ({angle_45_range:.2f}m)')
    
    plt.xlabel('horizon distance (m)')
    plt.ylabel('vec distance (m)')
    plt.title(f'best trajectory\nbest degrees: {optimal_angle:.2f}degrees, max range: {max_range:.2f}m')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 子图2: 区间收敛历史
    plt.subplot(2, 2, 2)
    iterations = [hist['iteration'] for hist in convergence_history]
    lows = [hist['low'] for hist in convergence_history]
    highs = [hist['high'] for hist in convergence_history]
    
    plt.fill_between(iterations, lows, highs, alpha=0.3, label='search range')
    plt.plot(iterations, lows, 'bo-', )
    plt.plot(iterations, highs, 'ro-',)
    plt.axhline(y=optimal_angle, color='green', linestyle='--', label=f'best angle ({optimal_angle:.2f}degrees)')
    plt.xlabel('n')
    plt.ylabel('degrees')
    plt.title('ternary_search history')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 子图3: 射程函数曲线
    plt.subplot(2, 2, 3)
    angles_test = np.linspace(30, 50, 100)
    ranges_test = [calculate_range(angle)[0] for angle in angles_test]
    
    plt.plot(angles_test, ranges_test, 'b-', label='R(θ)')
    plt.axvline(x=optimal_angle, color='red', linestyle='--', label=f'best angle ({optimal_angle:.2f}°)')
    plt.axvline(x=45, color='green', linestyle='--', label='45degrees (withour air)')
    
    plt.xlabel('shoot angle (°)')
    plt.ylabel('range (m)')
    plt.title(' range (shoot angle)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 子图4: 收敛速度分析
    plt.subplot(2, 2, 4)
    interval_lengths = [hist['high'] - hist['low'] for hist in convergence_history]
    theoretical_reduction = [(50-30) * (2/3)**i for i in range(len(interval_lengths))]
    
    plt.plot(iterations, interval_lengths, 'bo-',)
    plt.plot(iterations, theoretical_reduction, 'r--', )
    plt.axhline(y=0.1, color='green', linestyle='--', )
    plt.xlabel('n')
    plt.ylabel('interval')
    plt.title('ternary_search speed')
    plt.yscale('log')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def plot_parameter_analysis():
    """绘制角度与速度、角度与空气阻力参数的关系"""
    plt.figure(figsize=(12, 5))
    
    # 子图1: 最优角度与初始速度的关系
    plt.subplot(1, 2, 1)
    v0_range = np.linspace(5, 30, 50)
    optimal_angles_v0 = [find_optimal_angle_for_params(v, k=0.05) for v in v0_range]
    
    plt.plot(v0_range, optimal_angles_v0, 'bo-', linewidth=2, markersize=4)
    plt.axhline(y=45, color='red', linestyle='--', label='45° (no air resistance)')
    plt.xlabel('Initial velocity v₀ (m/s)')
    plt.ylabel('Optimal angle (°)')
    plt.title('Optimal angle vs Initial velocity (k=0.05)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 子图2: 最优角度与空气阻力系数的关系
    plt.subplot(1, 2, 2)
    k_range = np.linspace(0, 0.2, 50)
    optimal_angles_k = [find_optimal_angle_for_params(v0=10, k=k) for k in k_range]
    
    plt.plot(k_range, optimal_angles_k, 'ro-', linewidth=2, markersize=4)
    plt.axhline(y=45, color='red', linestyle='--', label='45° (no air resistance)')
    plt.xlabel('Air resistance coefficient k')
    plt.ylabel('Optimal angle (°)')
    plt.title('Optimal angle vs Air resistance coefficient (v₀=10 m/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

# 主程序
if __name__ == "__main__":
    # 使用三分法寻找最优角度
    optimal_angle, max_range, optimal_trajectory, convergence_history = ternary_search_optimization()
    
    # 计算无空气阻力情况
    optimal_angle_no_air, max_range_no_air = compare_with_no_air_resistance()
    
    # 绘制原有结果
    plot_results(optimal_angle, max_range, optimal_trajectory, convergence_history)
    
    # 绘制新的参数分析图
    plot_parameter_analysis()
    
    # 显示空气阻力的影响
    print(f"\n空气阻力影响分析:")
    print(f"最优角度变化: {optimal_angle_no_air - optimal_angle:.4f}° (降低)")
    print(f"最大射程减少: {max_range_no_air - max_range:.4f}m ({((max_range_no_air - max_range)/max_range_no_air*100):.2f}%)")
    
    # 显示收敛历史表格
    print("\n三分法收敛历史:")
    print("迭代 |    区间下界   |    区间上界   |   区间长度   |  三分点1射程  |  三分点2射程  ")
    print("-" * 85)
    for hist in convergence_history:
        print(f"{hist['iteration']:3d} | {hist['low']:10.6f}° | {hist['high']:10.6f}° | {hist['high']-hist['low']:8.6f}° | {hist['range_mid1']:10.6f}m | {hist['range_mid2']:10.6f}m")