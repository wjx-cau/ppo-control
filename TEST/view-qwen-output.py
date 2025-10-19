#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
查看 QWEN-OUTPUT 运行结果
快速浏览和分析历史运行数据
"""

import os
import json
from datetime import datetime

OUTPUT_BASE_DIR = "/Users/wjx_macair/Desktop/code/TEST/QWEN-OUTPUT"


def list_runs():
    """列出所有运行记录"""
    if not os.path.exists(OUTPUT_BASE_DIR):
        print(f"❌ 输出目录不存在：{OUTPUT_BASE_DIR}")
        return []
    
    runs = []
    for item in os.listdir(OUTPUT_BASE_DIR):
        path = os.path.join(OUTPUT_BASE_DIR, item)
        if os.path.isdir(path) and item.startswith("run_"):
            # 提取时间戳
            timestamp_str = item.replace("run_", "").replace("_test", "")
            try:
                # 尝试解析时间戳
                if len(timestamp_str) >= 15:
                    timestamp = datetime.strptime(timestamp_str[:15], "%Y%m%d_%H%M%S")
                    runs.append((timestamp, item, path))
            except ValueError:
                runs.append((None, item, path))
    
    return sorted(runs, reverse=True)  # 最新的在前


def view_run(run_path):
    """查看单次运行的详细信息"""
    run_name = os.path.basename(run_path)
    
    print(f"\n{'='*70}")
    print(f"📂 运行记录：{run_name}")
    print(f"{'='*70}\n")
    
    # 读取总结
    summary_path = os.path.join(run_path, "summary.txt")
    if os.path.exists(summary_path):
        print("📝 总结：")
        with open(summary_path, "r", encoding="utf-8") as f:
            print(f.read())
    else:
        print("⚠️ 未找到总结文件\n")
    
    # 列出所有迭代
    iterations = []
    for item in os.listdir(run_path):
        item_path = os.path.join(run_path, item)
        if os.path.isdir(item_path) and item.startswith("iteration_"):
            iterations.append((item, item_path))
    
    iterations.sort()
    
    if iterations:
        print(f"\n📊 迭代详情（共 {len(iterations)} 次）：")
        print(f"{'-'*70}\n")
        
        for iter_name, iter_path in iterations:
            iter_num = iter_name.replace("iteration_", "")
            print(f"🔄 迭代 {iter_num}:")
            
            # 读取分析结果
            analysis_path = os.path.join(iter_path, "analysis.json")
            if os.path.exists(analysis_path):
                with open(analysis_path, "r", encoding="utf-8") as f:
                    analysis = json.load(f)
                    summary = analysis.get("summary", "无总结")
                    control = analysis.get("control", {})
                    key = control.get("key", "?")
                    duration = control.get("duration", 0)
                    
                    print(f"   💬 观察：{summary}")
                    print(f"   🎮 控制：{key} 键 {duration:.1f}s")
            
            # 检查文件完整性
            files = os.listdir(iter_path)
            expected = ["camera_view.png", "robot_state.json", "analysis.json", "command.txt"]
            missing = [f for f in expected if f not in files]
            if missing:
                print(f"   ⚠️ 缺失文件：{', '.join(missing)}")
            
            print()
    else:
        print("⚠️ 未找到迭代数据\n")
    
    print(f"{'='*70}\n")


def main():
    print(f"\n{'='*70}")
    print(f"🔍 QWEN-OUTPUT 查看器")
    print(f"{'='*70}\n")
    
    runs = list_runs()
    
    if not runs:
        print("❌ 未找到任何运行记录")
        print(f"\n提示：运行 qwen-vision-control.py 后会在此目录生成数据：")
        print(f"      {OUTPUT_BASE_DIR}\n")
        return
    
    print(f"📁 找到 {len(runs)} 条运行记录：\n")
    
    for i, (timestamp, name, path) in enumerate(runs, 1):
        if timestamp:
            time_str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
            print(f"  {i}. {name} ({time_str})")
        else:
            print(f"  {i}. {name}")
    
    print(f"\n{'─'*70}")
    
    # 交互式选择
    try:
        choice = input("\n输入序号查看详情，或按 Enter 查看最新记录: ").strip()
        
        if choice == "":
            # 查看最新记录
            _, _, run_path = runs[0]
            view_run(run_path)
        elif choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(runs):
                _, _, run_path = runs[idx]
                view_run(run_path)
            else:
                print("❌ 无效的序号")
        else:
            print("❌ 无效的输入")
    
    except KeyboardInterrupt:
        print("\n\n👋 已退出")
    except Exception as e:
        print(f"\n❌ 错误：{e}")


if __name__ == "__main__":
    main()
