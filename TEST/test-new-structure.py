#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试新的 QWEN-OUTPUT 文件夹结构
验证每次迭代的数据是否正确保存
"""

import os
import json
import time

OUTPUT_BASE_DIR = "/Users/wjx_macair/Desktop/code/TEST/QWEN-OUTPUT"

def test_output_structure():
    """测试输出文件夹结构"""
    print(f"\n{'='*60}")
    print(f"测试 QWEN-OUTPUT 文件夹结构")
    print(f"{'='*60}\n")
    
    # 创建测试运行目录
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    test_run_dir = os.path.join(OUTPUT_BASE_DIR, f"run_{timestamp}_test")
    os.makedirs(test_run_dir, exist_ok=True)
    print(f"✅ 创建运行目录：{test_run_dir}")
    
    # 模拟3次迭代
    for iteration in range(1, 4):
        print(f"\n🔄 模拟迭代 {iteration}")
        
        # 创建迭代子目录
        iter_dir = os.path.join(test_run_dir, f"iteration_{iteration:02d}")
        os.makedirs(iter_dir, exist_ok=True)
        print(f"   📁 创建迭代目录：{iter_dir}")
        
        # 模拟保存摄像头图像
        camera_image_path = os.path.join(iter_dir, "camera_view.png")
        with open(camera_image_path, "w") as f:
            f.write("[模拟图像数据]")
        print(f"   📸 保存图像：camera_view.png")
        
        # 模拟保存机器人状态
        robot_state = {
            "iteration": iteration,
            "lift_height": 0.2 + iteration * 0.1,
            "j1_angle": iteration * 30,
            "j2_angle": 20,
            "j3_angle": -20
        }
        state_path = os.path.join(iter_dir, "robot_state.json")
        with open(state_path, "w", encoding="utf-8") as f:
            json.dump(robot_state, f, indent=2, ensure_ascii=False)
        print(f"   🤖 保存状态：robot_state.json")
        
        # 模拟保存分析结果
        analysis = {
            "summary": f"第{iteration}次观察：看到绿色植物",
            "control": {
                "key": "K",
                "duration": 0.3
            }
        }
        analysis_path = os.path.join(iter_dir, "analysis.json")
        with open(analysis_path, "w", encoding="utf-8") as f:
            json.dump(analysis, f, indent=2, ensure_ascii=False)
        print(f"   🧠 保存分析：analysis.json")
        
        # 模拟保存控制命令
        command_path = os.path.join(iter_dir, "command.txt")
        with open(command_path, "w", encoding="utf-8") as f:
            f.write(f"Key: K\n")
            f.write(f"Duration: 0.3\n")
            f.write(f"Summary: {analysis['summary']}\n")
        print(f"   🎮 保存命令：command.txt")
    
    # 模拟保存总结
    summary_path = os.path.join(test_run_dir, "summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("探索总结\n")
        f.write(f"{'='*60}\n")
        f.write(f"总观察次数: 3\n\n")
        f.write(f"观察历程：\n")
        f.write(f"1. 第1次观察：看到绿色植物 → K 0.3s\n")
        f.write(f"2. 第2次观察：看到绿色植物 → K 0.3s\n")
        f.write(f"3. 第3次观察：看到绿色植物 → K 0.3s\n")
    print(f"\n📝 保存总结：summary.txt")
    
    # 显示文件树
    print(f"\n{'='*60}")
    print(f"📁 生成的文件结构：")
    print(f"{'='*60}")
    print_tree(test_run_dir)
    
    print(f"\n✅ 测试完成！")
    print(f"📂 测试目录：{test_run_dir}\n")


def print_tree(directory, prefix="", max_depth=3, current_depth=0):
    """打印目录树"""
    if current_depth >= max_depth:
        return
    
    try:
        items = sorted(os.listdir(directory))
        for i, item in enumerate(items):
            path = os.path.join(directory, item)
            is_last = (i == len(items) - 1)
            
            connector = "└── " if is_last else "├── "
            print(f"{prefix}{connector}{item}")
            
            if os.path.isdir(path):
                extension = "    " if is_last else "│   "
                print_tree(path, prefix + extension, max_depth, current_depth + 1)
    except PermissionError:
        pass


if __name__ == "__main__":
    test_output_structure()
