# PyBullet机械臂仿真环境详细说明（README.ed）

## 目录
1. 项目简介
2. 主要功能
3. 依赖环境
4. 文件结构说明
5. 运行方法
6. 控制方式
7. 关键技术细节
8. 常见问题与调参建议
9. 致谢

---

## 1. 项目简介
本项目基于PyBullet物理引擎，构建了一个可交互的4自由度机械臂仿真环境，支持轨道移动、升降、三关节旋转、真实植株碰撞、末端摄像头等功能。适用于强化学习、控制算法测试、仿真可视化等场景。

## 2. 主要功能
- 机械臂4自由度（1升降+3旋转）
- 圆形轨道移动（Q/P键）
- 升降平台（Y/H键，0~0.5m）
- 关节无限/有限旋转（I/K, J/L, U/O）
- 真实植株模型（7叶片+茎+土壤）
- 末端摄像头（C键切换，V键调帧率）
- 物理参数可调，支持高精度仿真
- 强化学习训练与测试接口

## 3. 依赖环境
- Python 3.8+
- pybullet >= 3.2.5
- numpy, gymnasium, stable-baselines3（RL部分）
- 推荐Mac M系列芯片，Metal渲染器

## 4. 文件结构说明
```
pybullt/
  test_environment.py    # 主仿真交互程序
  rl_train.py            # 强化学习训练脚本
  test_slow.py           # 慢速演示脚本
  test_keys.py           # 键盘测试
  ppo_arm_final.zip      # 训练好的PPO模型
  vec_normalize.pkl      # RL归一化参数
  CAMERA_README.md       # 摄像头说明
  checkpoints/           # RL训练中间模型
  img/                   # 截图
  ppo_arm_tensorboard/   # RL训练日志
```

## 5. 运行方法
1. 安装依赖：
   ```sh
   pip install pybullet numpy gymnasium stable-baselines3
   ```
2. 进入目录并运行：
   ```sh
   cd pybullt
   python " test_environment.py"
   ```
3. 按提示操作，切换交互/演示模式。

## 6. 控制方式
- **升降**：Y（上升），H（下降），点按/长按均可
- **关节1**（底座）：I（逆时针），K（顺时针）
- **关节2**（肩部）：J（上），L（下）
- **关节3**（肘部）：U（上），O（下）
- **轨道移动**：Q（逆时针），P（顺时针）
- **摄像头**：C（开/关），V（调节帧率）
- **调试**：G（键盘日志），T（打印轨道角度）

## 7. 关键技术细节
- **URDF结构**：机械臂采用1个prismatic+3个revolute/continuous关节，支持无限旋转和升降。
- **物理参数**：所有关节阻尼、摩擦、力矩、速度均可调，已针对Mac M系列优化。
- **升降同步机制**：定期与物理引擎实际位置对齐，消除累计误差。
- **关节PD控制**：所有关节采用PyBullet内置PD控制，参数已调优，响应快且无惯性。
- **摄像头**：末端固定，支持160x120分辨率，帧率可调，Metal加速。
- **强化学习**：集成Stable-Baselines3 PPO算法，训练脚本和模型齐全。

## 8. 常见问题与调参建议
- **升降卡顿/抖动**：降低positionGain，增加damping。
- **关节惯性大**：提高velocityGain和URDF阻尼。
- **摄像头卡顿**：降低分辨率或提高frame_skip。
- **RL训练不收敛**：适当减少动作空间、增加奖励稀疏度。
- **按键无效**：确保已关闭PyBullet默认快捷键（已自动关闭）。

## 9. 致谢
- PyBullet官方文档与社区
- Stable-Baselines3 RL库
- Apple Metal团队
- 所有测试与反馈的同学

---
如有问题请联系作者或提交issue。
