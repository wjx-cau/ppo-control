#!/bin/bash
# 慢速测试训练好的强化学习模型

cd /Users/wjx_macair/Desktop/code/TEST/pybullt

# 使用 Python 运行内联脚本来控制速度
/Users/wjx_macair/miniforge/envs/bullet312/bin/python << 'PYTHON_SCRIPT'
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import pybullet as p
from rl_train import ArmReachEnv

print("=" * 60)
print("慢速演示 - 训练好的强化学习智能体")
print("=" * 60)

# 加载模型
model = PPO.load("ppo_arm_final")
print("✓ 模型加载成功")

# 创建环境
env = ArmReachEnv(render_mode="human", max_steps=500)
env_vec = VecNormalize.load("vec_normalize.pkl", DummyVecEnv([lambda: env]))
env_vec.training = False
env_vec.norm_reward = False
print("✓ 归一化参数加载成功\n")

# 运行3个回合
for episode in range(3):
    print(f"\n{'='*60}")
    print(f"第 {episode + 1}/3 回合")
    print(f"{'='*60}")
    print("👀 观察机械臂如何自主接触绿色叶片...")
    
    obs = env_vec.reset()
    done = False
    steps = 0
    text_id = None
    
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env_vec.step(action)
        steps += 1
        
        # 获取并显示距离
        dist = env._get_distance_to_target()
        if text_id is not None:
            try:
                p.removeUserDebugItem(text_id)
            except:
                pass
        try:
            text_id = p.addUserDebugText(
                f"步数: {steps} | 距离: {dist:.3f}m",
                [0.3, 0.3, 0.6],
                textColorRGB=[1, 1, 0],
                textSize=1.8
            )
        except:
            pass
        
        if steps % 5 == 0:
            print(f"  步骤 {steps:3d}: 距离 {dist:.4f}m")
        
        # 慢速演示 - 关键！
        time.sleep(0.08)  # 80ms每步，清晰可见
        
        # 检查成功
        info = info[0] if isinstance(info, list) else info
        if info.get("success", False):
            if text_id:
                try:
                    p.removeUserDebugItem(text_id)
                except:
                    pass
            try:
                p.addUserDebugText("🎉 成功！", [0.35, 0.0, 0.5], textColorRGB=[0, 1, 0], textSize=2.5)
            except:
                pass
            print(f"\n🎉 成功！步数: {steps}")
            time.sleep(2)
            break
    
    if episode < 2:
        print("\n⏳ 3秒后开始下一回合...")
        time.sleep(3)

env_vec.close()
print("\n演示结束！")
PYTHON_SCRIPT

