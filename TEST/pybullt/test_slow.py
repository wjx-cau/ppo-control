#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
慢速演示测试脚本 - 清晰展示训练好的智能体
"""

import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import pybullet as p

# 导入环境（假设在同一目录）
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from rl_train_backup import ArmReachEnv


def test_slow(model_path="ppo_arm_final", episodes=3):
    """慢速测试已训练的智能体"""
    print("=" * 60)
    print("慢速演示 - 训练好的强化学习智能体")
    print("=" * 60)
    
    # 加载模型
    print(f"\n加载模型：{model_path}")
    try:
        model = PPO.load(model_path)
        print("✓ 模型加载成功")
    except FileNotFoundError:
        print(f"✗ 模型文件未找到：{model_path}.zip")
        return
    
    # 创建可视化环境（降低max_steps让演示更快完成）
    env = ArmReachEnv(render_mode="human", max_steps=500)
    
    # 加载归一化参数
    try:
        env_vec = VecNormalize.load("vec_normalize.pkl", DummyVecEnv([lambda: env]))
        env_vec.training = False
        env_vec.norm_reward = False
        print("✓ 归一化参数加载成功\n")
        use_vec = True
    except FileNotFoundError:
        print("⚠ 未找到归一化参数，使用未归一化的环境\n")
        use_vec = False
    
    # 运行测试回合
    for episode in range(episodes):
        print("\n" + "=" * 60)
        print(f"第 {episode + 1}/{episodes} 回合")
        print("=" * 60)
        print("👀 观察机械臂如何自主学习接触绿色叶片...\n")
        
        if use_vec:
            obs = env_vec.reset()
        else:
            obs = env.reset()[0]
            obs = np.array([obs])
        
        done = False
        total_reward = 0
        steps = 0
        text_id = None
        
        while not done:
            # 预测动作
            action, _ = model.predict(obs, deterministic=True)
            
            # 执行动作
            if use_vec:
                obs, reward, done, info = env_vec.step(action)
                info = info[0] if isinstance(info, list) else info
                reward_val = float(reward[0]) if isinstance(reward, np.ndarray) else float(reward)
            else:
                obs_raw, reward_val, terminated, truncated, info = env.step(action[0])
                obs = np.array([obs_raw])
                done = terminated or truncated
            
            total_reward += reward_val
            steps += 1
            
            # 获取距离信息
            try:
                dist = env._get_distance_to_target()
                status_text = f"步数: {steps} | 距离目标: {dist:.3f}m"
                
                # 更新屏幕文字
                if text_id is not None:
                    try:
                        p.removeUserDebugItem(text_id)
                    except:
                        pass
                
                try:
                    text_id = p.addUserDebugText(
                        status_text,
                        [0.3, 0.3, 0.6],
                        textColorRGB=[1, 1, 0],
                        textSize=1.8
                    )
                except:
                    pass
                
                # 控制台输出
                if steps % 5 == 0:  # 每5步输出一次
                    print(f"  步骤 {steps:3d}: 距离 {dist:.4f}m")
            except:
                pass
            
            # 慢速演示：每步暂停更长时间
            time.sleep(0.05)  # 50ms 每步，让动作清晰可见
            
            # 检查是否成功
            if info.get("success", False):
                # 清除文字
                if text_id is not None:
                    try:
                        p.removeUserDebugItem(text_id)
                    except:
                        pass
                
                # 显示成功信息
                try:
                    success_text_id = p.addUserDebugText(
                        "🎉 成功接触！",
                        [0.35, 0.0, 0.5],
                        textColorRGB=[0, 1, 0],
                        textSize=2.5
                    )
                except:
                    pass
                
                print(f"\n{'='*60}")
                print("🎉 成功！")
                print(f"{'='*60}")
                print(f"  步数: {steps}")
                print(f"  总奖励: {total_reward:.2f}")
                print(f"  末端成功接触到绿色叶片！")
                print(f"{'='*60}")
                
                time.sleep(2)  # 成功后暂停2秒
                
                try:
                    p.removeUserDebugItem(success_text_id)
                except:
                    pass
                break
        
        if not info.get("success", False):
            dist = info.get('distance', 'N/A')
            print(f"\n{'='*60}")
            print("❌ 未成功")
            print(f"{'='*60}")
            print(f"  步数: {steps}")
            print(f"  总奖励: {total_reward:.2f}")
            if dist != 'N/A':
                print(f"  最终距离: {dist:.3f}m")
            print(f"{'='*60}")
        
        # 清除文字
        if text_id is not None:
            try:
                p.removeUserDebugItem(text_id)
            except:
                pass
        
        # 回合间隔
        if episode < episodes - 1:
            print("\n⏳ 3秒后开始下一回合...")
            time.sleep(3)
    
    # 关闭环境
    if use_vec:
        env_vec.close()
    else:
        env.close()
    
    print("\n" + "=" * 60)
    print("演示结束！")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="慢速演示训练好的智能体")
    parser.add_argument("--model", type=str, default="ppo_arm_final", help="模型路径")
    parser.add_argument("--episodes", type=int, default=3, help="测试回合数")
    args = parser.parse_args()
    
    test_slow(args.model, args.episodes)
