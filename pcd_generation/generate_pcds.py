#!/usr/bin/env python3
"""
在虚拟环境中生成重复场景点云文件
"""

import open3d as o3d
import numpy as np
import os
import sys

def print_progress_bar(iteration, total, prefix='', suffix='', length=50, fill='█'):
    """
    打印进度条
    """
    percent = f"{100 * (iteration / float(total)):.1f}"
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    sys.stdout.write(f'\r{prefix} |{bar}| {percent}% {suffix}')
    sys.stdout.flush()
    if iteration == total:
        print()

def create_corridor_pointcloud(filename="corridor_scene.pcd"):
    """
    创建长直走廊点云
    """
    print(f"生成走廊场景: {filename}")
    points = []
    
    # 走廊参数
    corridor_length = 40.0
    corridor_width = 6.0
    corridor_height = 5.0
    
    # 1. 地面（大网格点）
    ground_count = 0
    for x in np.arange(-corridor_length/2, corridor_length/2, 0.15):
        for y in np.arange(-corridor_width/2, corridor_width/2, 0.15):
            points.append([x, y, 0.0])
            ground_count += 1
    
    print(f"  地面点数: {ground_count}")
    
    # 2. 左侧墙面
    left_wall_count = 0
    for x in np.arange(-corridor_length/2, corridor_length/2, 0.1):
        for z in np.arange(0, corridor_height, 0.1):
            points.append([x, -corridor_width/2, z])
            left_wall_count += 1
    
    # 3. 右侧墙面
    right_wall_count = 0
    for x in np.arange(-corridor_length/2, corridor_length/2, 0.1):
        for z in np.arange(0, corridor_height, 0.1):
            points.append([x, corridor_width/2, z])
            right_wall_count += 1
    
    print(f"  墙面点数: {left_wall_count + right_wall_count}")
    
    # 4. 重复的柱子
    pillar_interval = 5.0
    pillar_radius = 0.3
    pillar_height = 3.0
    pillar_count = 0
    
    num_pillars = int(corridor_length / pillar_interval)
    
    for i in range(num_pillars):
        x = -corridor_length/2 + 2.5 + i * pillar_interval
        
        # 左侧柱子
        for angle in np.arange(0, 2*np.pi, 0.15):
            for z in np.arange(0, pillar_height, 0.15):
                y = -corridor_width/2 + 1.0 + pillar_radius * np.cos(angle)
                points.append([x, y, z])
                pillar_count += 1
        
        # 右侧柱子
        for angle in np.arange(0, 2*np.pi, 0.15):
            for z in np.arange(0, pillar_height, 0.15):
                y = corridor_width/2 - 1.0 + pillar_radius * np.cos(angle)
                points.append([x, y, z])
                pillar_count += 1
        
        print_progress_bar(i+1, num_pillars, prefix='  柱子生成进度:', suffix=f'{i+1}/{num_pillars}')
    
    print(f"  柱子点数: {pillar_count}")
    
    # 转换为点云
    points_array = np.array(points)
    
    # 添加轻微噪声（可选）
    if len(points_array) > 0:
        noise = np.random.normal(0, 0.005, points_array.shape)  # 5mm噪声
        points_array += noise
    
    # 创建Open3D点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_array)
    
    # 为不同部分添加颜色（可视化用）
    colors = []
    idx = 0
    
    # 地面 - 深灰色
    for _ in range(ground_count):
        colors.append([0.3, 0.3, 0.3])
        idx += 1
    
    # 墙面 - 中灰色
    for _ in range(left_wall_count + right_wall_count):
        colors.append([0.5, 0.5, 0.5])
        idx += 1
    
    # 柱子 - 浅灰色
    for _ in range(pillar_count):
        colors.append([0.7, 0.7, 0.7])
        idx += 1
    
    pcd.colors = o3d.utility.Vector3dVector(colors[:len(points_array)])
    
    # 保存
    o3d.io.write_point_cloud(filename, pcd)
    print(f"  总点数: {len(points_array)}")
    print(f"  保存到: {os.path.abspath(filename)}\n")
    
    return filename

def create_symmetric_room_pcd(filename="symmetric_room.pcd"):
    """
    创建对称房间点云
    """
    print(f"生成对称房间场景: {filename}")
    points = []
    
    # 房间尺寸
    room_size = 20.0
    wall_height = 4.0
    
    # 1. 地面
    ground_count = 0
    for x in np.arange(-room_size/2, room_size/2, 0.15):
        for y in np.arange(-room_size/2, room_size/2, 0.15):
            points.append([x, y, 0.0])
            ground_count += 1
    
    print(f"  地面点数: {ground_count}")
    
    # 2. 四面墙
    wall_count = 0
    # 东墙
    for y in np.arange(-room_size/2, room_size/2, 0.1):
        for z in np.arange(0, wall_height, 0.1):
            points.append([room_size/2, y, z])
            wall_count += 1
    
    # 西墙
    for y in np.arange(-room_size/2, room_size/2, 0.1):
        for z in np.arange(0, wall_height, 0.1):
            points.append([-room_size/2, y, z])
            wall_count += 1
    
    # 北墙
    for x in np.arange(-room_size/2, room_size/2, 0.1):
        for z in np.arange(0, wall_height, 0.1):
            points.append([x, room_size/2, z])
            wall_count += 1
    
    # 南墙
    for x in np.arange(-room_size/2, room_size/2, 0.1):
        for z in np.arange(0, wall_height, 0.1):
            points.append([x, -room_size/2, z])
            wall_count += 1
    
    print(f"  墙面点数: {wall_count}")
    
    # 3. 对称柱子（四个角落）
    pillar_positions = [
        (-7, -7), (-7, 7), (7, -7), (7, 7)
    ]
    pillar_count = 0
    
    for i, (px, py) in enumerate(pillar_positions):
        for angle in np.arange(0, 2*np.pi, 0.15):
            for z in np.arange(0, 3.0, 0.15):
                points.append([
                    px + 0.5 * np.cos(angle),
                    py + 0.5 * np.sin(angle),
                    z
                ])
                pillar_count += 1
        print_progress_bar(i+1, 4, prefix='  柱子生成进度:', suffix=f'{i+1}/4')
    
    print(f"  柱子点数: {pillar_count}")
    
    # 4. 中心立方体
    cube_size = 4.0
    cube_count = 0
    for x in np.arange(-cube_size/2, cube_size/2, 0.2):
        for y in np.arange(-cube_size/2, cube_size/2, 0.2):
            for z in np.arange(0, cube_size, 0.2):
                # 避免中心点太密
                if np.random.random() > 0.7:
                    points.append([x, y, z])
                    cube_count += 1
    
    print(f"  中心立方体点数: {cube_count}")
    
    # 转换为点云
    points_array = np.array(points)
    
    # 添加颜色
    colors = []
    idx = 0
    
    # 地面 - 深灰色
    for _ in range(ground_count):
        colors.append([0.3, 0.3, 0.3])
        idx += 1
    
    # 墙面 - 中灰色
    for _ in range(wall_count):
        colors.append([0.5, 0.5, 0.5])
        idx += 1
    
    # 柱子 - 红色
    for _ in range(pillar_count):
        colors.append([0.8, 0.2, 0.2])
        idx += 1
    
    # 中心立方体 - 绿色
    for _ in range(cube_count):
        colors.append([0.2, 0.8, 0.2])
        idx += 1
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_array)
    pcd.colors = o3d.utility.Vector3dVector(colors[:len(points_array)])
    
    # 保存
    o3d.io.write_point_cloud(filename, pcd)
    print(f"  总点数: {len(points_array)}")
    print(f"  保存到: {os.path.abspath(filename)}\n")
    
    return filename

def create_degenerate_corridor_pcd(filename="degenerate_corridor.pcd"):
    """
    创建高度退化的走廊（几乎没有特征）
    用于测试SLAM在特征稀少环境中的表现
    """
    print(f"生成退化走廊场景: {filename}")
    points = []
    
    # 非常长的直走廊，几乎没有特征
    corridor_length = 100.0  # 100米长
    corridor_width = 5.0
    corridor_height = 4.0
    
    # 1. 光滑地面（非常稀疏的点）
    ground_count = 0
    for x in np.arange(-corridor_length/2, corridor_length/2, 0.5):  # 非常稀疏
        for y in np.arange(-corridor_width/2, corridor_width/2, 0.5):
            points.append([x, y, 0.0])
            ground_count += 1
    
    # 2. 光滑墙面（几乎没有特征）
    wall_count = 0
    for x in np.arange(-corridor_length/2, corridor_length/2, 0.3):
        # 左侧墙面
        for z in np.arange(0, corridor_height, 0.3):
            points.append([x, -corridor_width/2, z])
            wall_count += 1
        
        # 右侧墙面
        for z in np.arange(0, corridor_height, 0.3):
            points.append([x, corridor_width/2, z])
            wall_count += 1
    
    # 3. 几乎没有任何障碍物或特征点
    
    points_array = np.array(points)
    
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_array)
    
    # 所有点都设为相似的颜色（增加退化程度）
    colors = np.ones((len(points_array), 3)) * 0.6
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 保存
    o3d.io.write_point_cloud(filename, pcd)
    print(f"  总点数: {len(points_array)}")
    print(f"  保存到: {os.path.abspath(filename)}\n")
    print("  注意：这是一个高度退化的场景，用于测试SLAM在特征稀少环境中的表现\n")
    
    return filename

def create_tunnel_pcd(filename="tunnel_scene.pcd"):
    """
    创建圆形隧道场景（退化场景）
    """
    print(f"生成隧道场景: {filename}")
    points = []
    
    # 隧道参数
    tunnel_length = 50.0
    tunnel_radius = 3.0
    
    # 创建隧道壁（圆形截面）
    point_count = 0
    
    # 沿隧道长度方向
    for t in np.arange(0, tunnel_length, 0.2):
        x = t - tunnel_length/2
        
        # 圆形截面
        for angle in np.arange(0, 2*np.pi, 0.15):
            y = tunnel_radius * np.cos(angle)
            z = tunnel_radius * np.sin(angle) + tunnel_radius
            
            points.append([x, y, z])
            point_count += 1
        
        print_progress_bar(int(t/0.2), int(tunnel_length/0.2), 
                          prefix='  隧道生成进度:', 
                          suffix=f'{int(t/0.2)}/{int(tunnel_length/0.2)}')
    
    points_array = np.array(points)
    
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_array)
    
    # 保存
    o3d.io.write_point_cloud(filename, pcd)
    print(f"  总点数: {len(points_array)}")
    print(f"  保存到: {os.path.abspath(filename)}\n")
    
    return filename

def create_sparse_forest_pcd(filename="sparse_forest.pcd"):
    """
    创建稀疏森林场景
    """
    print(f"生成稀疏森林场景: {filename}")
    points = []
    
    # 网格参数
    grid_size = 12  # 12x12网格
    spacing = 4.0   # 树间距4米
    
    # 地面
    ground_count = 0
    for x in np.arange(-grid_size/2*spacing, grid_size/2*spacing, 0.3):
        for y in np.arange(-grid_size/2*spacing, grid_size/2*spacing, 0.3):
            points.append([x, y, 0.0])
            ground_count += 1
    
    # 树木
    tree_count = 0
    tree_idx = 0
    total_trees = grid_size * grid_size
    
    for i in range(grid_size):
        for j in range(grid_size):
            # 随机跳过一些位置
            if np.random.random() < 0.3:  # 30%的位置没有树
                continue
                
            x = (i - grid_size/2) * spacing + np.random.uniform(-0.5, 0.5)
            y = (j - grid_size/2) * spacing + np.random.uniform(-0.5, 0.5)
            
            tree_count += 1
            
            # 树干（圆柱体）
            trunk_height = np.random.uniform(2.0, 4.0)
            trunk_radius = np.random.uniform(0.2, 0.4)
            
            for angle in np.arange(0, 2*np.pi, 0.2):
                for z in np.arange(0, trunk_height, 0.2):
                    points.append([
                        x + trunk_radius * np.cos(angle),
                        y + trunk_radius * np.sin(angle),
                        z
                    ])
            
            # 树冠（球体）
            canopy_radius = np.random.uniform(1.0, 2.0)
            for angle1 in np.arange(0, np.pi, 0.3):
                for angle2 in np.arange(0, 2*np.pi, 0.3):
                    radius = canopy_radius * np.sin(angle1)
                    if radius > 0.1:  # 避免中心点太密
                        points.append([
                            x + radius * np.cos(angle2),
                            y + radius * np.sin(angle2),
                            trunk_height + canopy_radius * np.cos(angle1)
                        ])
            
            tree_idx += 1
            print_progress_bar(tree_idx, total_trees, 
                              prefix='  树木生成进度:', 
                              suffix=f'{tree_idx}/{total_trees}')
    
    points_array = np.array(points)
    
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_array)
    
    # 保存
    o3d.io.write_point_cloud(filename, pcd)
    print(f"\n  地面点数: {ground_count}")
    print(f"  树木数量: {tree_count}")
    print(f"  总点数: {len(points_array)}")
    print(f"  保存到: {os.path.abspath(filename)}\n")
    
    return filename

def main():
    """主函数"""
    print("=" * 60)
    print("重复场景点云生成工具")
    print("使用虚拟环境以避免污染系统Python环境")
    print("=" * 60)
    
    # 检查是否在虚拟环境中
    if sys.prefix == sys.base_prefix:
        print("警告: 不在虚拟环境中运行!")
        print("建议先激活虚拟环境:")
        print("  source venv/bin/activate")
        response = input("是否继续? (y/n): ")
        if response.lower() != 'y':
            print("退出...")
            return
    
    # 创建输出目录
    output_dir = "generated_pcds"
    os.makedirs(output_dir, exist_ok=True)
    print(f"点云文件将保存到: {os.path.abspath(output_dir)}")
    
    # 生成各种场景
    scenes = [
        ("走廊场景", create_corridor_pointcloud, "corridor_scene.pcd"),
        ("对称房间", create_symmetric_room_pcd, "symmetric_room.pcd"),
        ("退化走廊", create_degenerate_corridor_pcd, "degenerate_corridor.pcd"),
        ("隧道场景", create_tunnel_pcd, "tunnel_scene.pcd"),
        ("稀疏森林", create_sparse_forest_pcd, "sparse_forest.pcd"),
    ]
    
    generated_files = []
    
    for scene_name, generator_func, filename in scenes:
        print(f"\n{'='*40}")
        print(f"生成: {scene_name}")
        print(f"{'='*40}")
        
        filepath = os.path.join(output_dir, filename)
        try:
            result_file = generator_func(filepath)
            generated_files.append(result_file)
        except Exception as e:
            print(f"  错误: {e}")
            print(f"  跳过 {scene_name}")
    
    # 总结
    print("\n" + "=" * 60)
    print("点云生成完成!")
    print("=" * 60)
    
    print(f"\n生成的文件 ({len(generated_files)} 个):")
    for i, filepath in enumerate(generated_files, 1):
        file_size = os.path.getsize(filepath) / (1024*1024)  # MB
        print(f"  {i}. {os.path.basename(filepath)} ({file_size:.2f} MB)")
    
    print(f"\n所有文件保存在: {os.path.abspath(output_dir)}")
    
    # 复制到MARSIM资源目录的指令
    print("\n下一步: 将PCD文件复制到MARSIM资源目录")
    print("命令示例:")
    print(f"  cp {output_dir}/*.pcd ~/catkin_ws/src/map_generator/resource/")
    
    # 检查目标目录是否存在
    target_dir = os.path.expanduser("~/catkin_ws/src/map_generator/resource")
    if os.path.exists(target_dir):
        print(f"目标目录存在: {target_dir}")
    else:
        print(f"警告: 目标目录不存在: {target_dir}")
        print("请确保MARSIM已正确安装")

if __name__ == "__main__":
    main()
