# 🤖 强化学习机械臂控制系统

<div align="center">

**使用 PPO 算法训练 3-DoF 机械臂自主接触目标叶片**

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![PyBullet](https://img.shields.io/badge/PyBullet-3.2.5+-green.svg)](https://pybullet.org/)
[![Stable-Baselines3](https://img.shields.io/badge/SB3-2.0+-orange.svg)](https://stable-baselines3.readthedocs.io/)

</div>

---

## 📋 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [详细使用指南](#详细使用指南)
- [环境设计](#环境设计)
- [训练技巧](#训练技巧)
- [结果分析](#结果分析)
- [常见问题](#常见问题)
- [进阶优化](#进阶优化)
- [文件结构](#文件结构)

---

## 🎯 项目简介

本项目实现了一个基于**深度强化学习**的 3 自由度机械臂控制系统，使用 **PPO (Proximal Policy Optimization)** 算法训练智能体，让机械臂学会自主规划轨迹并精准接触目标叶片。

![demo](assets/img/1)

### 核心技术栈

- **物理引擎**: PyBullet（实时刚体物理仿真）
- **强化学习框架**: Stable-Baselines3
- **深度学习**: PyTorch（神经网络后端）
- **环境接口**: Gymnasium（OpenAI Gym 标准）

### 应用场景

- 🌱 农业机器人（采摘、修剪）

- 🏭 工业装配（精准定位）

- 🔬 实验室自动化（样本操作）

  ![截屏2025-10-14 下午5.49.16](/Users/wjx_macair/Desktop/截屏2025-10-14 下午5.49.16.png)

- 📚 强化学习教学演示

---

## ✨ 功能特性

### ✅ 已实现功能

- ✅ **完整的 RL 训练流程**：从零开始训练到模型保存
- ✅ **实时可视化**：GUI 显示训练进度和测试效果
- ✅ **慢速演示模式**：清晰观察每一步决策过程
- ✅ **自动检查点保存**：每 10,000 步自动保存模型
- ✅ **TensorBoard 监控**：实时查看训练曲线
- ✅ **100% 成功率**：训练后智能体稳定完成任务
- ✅ **高效学习**：5-10 万步即可达到良好效果

### 🎮 交互功能

- 🖥️ **GUI 可视化**：实时显示步数、距离、成功状态
- 📊 **实时统计**：每 5 步输出距离变化
- 🎨 **颜色标注**：黄色状态、绿色成功提示
- ⏱️ **速度控制**：可调节演示速度（60Hz + 额外延迟）

---

## 💻 系统要求

### 硬件要求

- **CPU**: 任意现代 CPU（支持 x86_64 或 ARM）
- **内存**: 最低 4GB（推荐 8GB+）
- **GPU**: 可选（Apple Silicon 支持 MPS 加速）
- **存储**: 至少 500MB 空闲空间

### 软件要求

- **操作系统**: macOS / Linux / Windows
- **Python**: 3.8 - 3.12
- **Conda**: 推荐使用 Miniconda/Anaconda

### 支持的加速

- ✅ Apple M1/M2/M3/M4 (MPS)
- ✅ NVIDIA CUDA
- ✅ CPU（通用支持）

---

## 🚀 快速开始

### 1. 环境配置

#### 方式一：使用 Conda（推荐）

```bash
# 创建虚拟环境
conda create -n bullet312 python=3.12 -y
conda activate bullet312

# 安装依赖
pip install pybullet stable-baselines3 gymnasium tensorboard
```

#### 方式二：使用 pip

```bash
# 创建虚拟环境
python -m venv venv
source venv/bin/activate  # Linux/macOS
# venv\Scripts\activate  # Windows

# 安装依赖
pip install pybullet stable-baselines3 gymnasium tensorboard
```

### 2. 克隆/下载项目

```bash
cd /path/to/your/workspace
# 假设项目在 TEST/pybullt 目录
```

### 3. 快速训练（5 万步演示）

```bash
cd TEST/pybullt
python rl_train.py --train --timesteps 50000
```

**预期输出**：
```
============================================================
开始训练强化学习智能体
============================================================
关节数: 4, 末端链接索引: 3
Using cpu device

开始训练 50000 步...
Logging to ./ppo_arm_tensorboard/PPO_1
-----------------------------
| time/              |      |
|    fps             | 4589 |
|    iterations      | 1    |
|    time_elapsed    | 0    |
|    total_timesteps | 2048 |
-----------------------------
...
训练完成！模型已保存到 ppo_arm_final.zip
```

### 4. 测试训练好的模型

```bash
python rl_train.py --test --episodes 3
```

**预期结果**：
- 弹出 PyBullet GUI 窗口
- 机械臂自主移动接近绿色叶片
- 显示实时步数和距离
- 成功接触后显示绿色提示

---

## 📖 详细使用指南

### 训练模式

#### 基础训练

```bash
python rl_train.py --train --timesteps 200000
```

#### 参数说明

| 参数 | 说明 | 默认值 | 推荐值 |
|------|------|--------|--------|
| `--timesteps` | 总训练步数 | 200000 | 初学: 50000<br>标准: 200000<br>精调: 500000+ |

#### 训练输出

```
项目目录/
├── ppo_arm_final.zip           # 最终训练模型
├── vec_normalize.pkl           # 归一化参数
├── checkpoints/                # 中间检查点
│   ├── ppo_arm_10000_steps.zip
│   ├── ppo_arm_20000_steps.zip
│   └── ...
└── ppo_arm_tensorboard/        # TensorBoard 日志
    └── PPO_1/
        └── events.out.tfevents...
```

### 测试模式

#### 基础测试

```bash
python rl_train.py --test --episodes 5
```

#### 测试不同检查点

```bash
# 测试 10000 步的模型
python rl_train.py --test --model-path checkpoints/ppo_arm_10000_steps --episodes 3

# 测试 50000 步的模型
python rl_train.py --test --model-path checkpoints/ppo_arm_50000_steps --episodes 3
```

#### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--episodes` | 测试回合数 | 5 |
| `--model-path` | 模型路径 | `ppo_arm_final` |

### TensorBoard 监控

#### 启动 TensorBoard

```bash
tensorboard --logdir ./ppo_arm_tensorboard/
```

#### 访问界面

浏览器打开：http://localhost:6006

#### 可查看的指标

- **train/loss**: 策略损失
- **train/policy_gradient_loss**: 策略梯度损失
- **train/value_loss**: 价值函数损失
- **train/entropy_loss**: 熵损失（探索度）
- **train/approx_kl**: KL 散度（策略变化）
- **train/clip_fraction**: 裁剪比例
- **rollout/ep_rew_mean**: 平均回合奖励

---

## 🎨 环境设计

### 机械臂结构

```
        ┌──────┐
        │  EE  │ ← 末端执行器（红色小方块）
        └──┬───┘
           │
       ┌───┴────┐
       │ Link3  │ ← 第三段连杆（绿色，L=0.16m）
       └───┬────┘
           │ J3 (Y轴)
       ┌───┴────┐
       │ Link2  │ ← 第二段连杆（橙色，L=0.18m）
       └───┬────┘
           │ J2 (Y轴)
       ┌───┴────┐
       │ Link1  │ ← 第一段连杆（蓝色，L=0.22m）
       └───┬────┘
           │ J1 (Z轴)
       ┌───┴────┐
       │  Base  │ ← 基座（灰色，固定）
       └────────┘
```

### 关节限制

| 关节 | 轴向 | 范围（度） | 范围（弧度） | 用途 |
|------|------|-----------|-------------|------|
| J1 | Z | -180° ~ 180° | -π ~ π | 水平旋转（yaw） |
| J2 | Y | -100° ~ 100° | -1.745 ~ 1.745 | 俯仰（pitch） |
| J3 | Y | -120° ~ 120° | -2.094 ~ 2.094 | 俯仰（pitch） |

### 观察空间（13维）

```python
Observation = [
    joint_angle_1,    # J1 角度 (rad)
    joint_angle_2,    # J2 角度 (rad)
    joint_angle_3,    # J3 角度 (rad)
    joint_vel_1,      # J1 角速度 (rad/s)
    joint_vel_2,      # J2 角速度 (rad/s)
    joint_vel_3,      # J3 角速度 (rad/s)
    ee_pos_x,         # 末端 X 坐标 (m)
    ee_pos_y,         # 末端 Y 坐标 (m)
    ee_pos_z,         # 末端 Z 坐标 (m)
    target_x,         # 目标 X 坐标 (m) = 0.35
    target_y,         # 目标 Y 坐标 (m) = 0.0
    target_z,         # 目标 Z 坐标 (m) = 0.35
    distance          # 末端到目标的欧氏距离 (m)
]
```

### 动作空间（3维）

```python
Action = [
    delta_angle_1,    # J1 角度增量 (-0.1 ~ 0.1 rad)
    delta_angle_2,    # J2 角度增量 (-0.1 ~ 0.1 rad)
    delta_angle_3     # J3 角度增量 (-0.1 ~ 0.1 rad)
]
```

**注意**：动作是增量式的，每步在当前角度基础上调整。

### 奖励函数

#### 距离奖励（主要驱动力）

```python
distance_reward = -distance × 10.0
```

- 距离越小，奖励越高（惩罚越小）
- 鼓励智能体接近目标

#### 进步奖励（加速收敛）

```python
progress_reward = (prev_distance - curr_distance) × 50.0
```

- 每步更接近目标 → 获得额外奖励
- 每步远离目标 → 获得额外惩罚

#### 到达奖励（里程碑）

```python
if distance < 0.05:  # 5cm内
    reach_reward = 50.0
if distance < 0.02:  # 2cm内
    reach_reward = 100.0
```

#### 接触奖励（终极目标）

```python
if end_effector_touches_leaf:
    contact_reward = 100.0
    success = True
```

#### 总奖励

```python
total_reward = distance_reward + progress_reward + reach_reward + contact_reward
```

### 终止条件

1. **成功终止**：末端执行器接触到叶片
2. **超时终止**：达到最大步数（500 步）

---

## 🎓 训练技巧

### 初级阶段（0 - 5万步）

**目标**：学习基本运动控制

- ✅ 智能体开始理解关节如何影响末端位置
- ✅ 出现向目标方向的趋势
- ✅ 偶尔能接近目标区域

**特征**：
- 成功率：0% - 20%
- 平均步数：300+
- 轨迹：杂乱无章，大量探索

### 中级阶段（5万 - 15万步）

**目标**：提高成功率和效率

- ✅ 稳定接近目标
- ✅ 成功率显著提升
- ✅ 轨迹开始优化

**特征**：
- 成功率：20% - 70%
- 平均步数：100 - 200
- 轨迹：有明显规划，但还有抖动

### 高级阶段（15万 - 30万步）

**目标**：精调策略，提升稳定性

- ✅ 高成功率（> 80%）
- ✅ 快速完成（< 100 步）
- ✅ 平滑轨迹

**特征**：
- 成功率：70% - 100%
- 平均步数：50 - 100
- 轨迹：流畅直接

### 超参数调优建议

#### 学习率调整

```python
# 训练早期 - 快速学习
learning_rate = 3e-4  # 默认

# 微调阶段 - 稳定优化
learning_rate = 1e-4

# 精细调整
learning_rate = 3e-5
```

#### Batch Size 影响

```python
# 小 batch（更新频繁，适合初期）
batch_size = 32

# 中 batch（平衡）
batch_size = 64  # 默认

# 大 batch（稳定但慢）
batch_size = 128
```

#### 探索度控制

```python
# 高探索（训练初期）
ent_coef = 0.01  # 默认

# 低探索（收敛后）
ent_coef = 0.001
```

---

## 📊 结果分析

### 训练曲线解读

#### 理想曲线

```
奖励值
  │
  │     ┌────────────
  │    ╱
  │   ╱
  │  ╱
  │ ╱
  └─────────────────> 步数
  0  5万 10万 15万 20万
```

#### 实际表现（50000 步训练）

| 指标 | 数值 | 说明 |
|------|------|------|
| **训练步数** | 50,000 | 约需 15 分钟 |
| **成功率** | 100% | 3/3 测试回合全部成功 |
| **平均步数** | 87.7 | (50 + 142 + 71) / 3 |
| **最佳表现** | 50 步 | 第 1 回合 |
| **FPS** | 3,000+ | 训练速度（步/秒）|

### 测试案例分析



#### 🏆 回合 1（优秀）

```
步数: 50
距离变化: 0.44m → 0.30m → 0.17m → 0.23m (接触)
奖励: -67.61
特点: 快速直达，略有摆动
```

#### ⚠️ 回合 2（需优化）

```
步数: 142
距离变化: 起伏较大，在 0.15m - 0.55m 间振荡
奖励: -446.00
特点: 反复试探，最终成功
```

#### ✅ 回合 3（良好）

```
步数: 71
距离变化: 0.56m → 0.27m → 0.11m (接触)
奖励: -160.63
特点: 渐进式接近，较平滑
```

### 性能优化空间

当前模型（50,000 步）已经可用，但还有提升空间：

1. **继续训练** → 200,000 步可达到更高稳定性
2. **调整奖励权重** → 减少振荡
3. **增加动作平滑惩罚** → 更优雅的轨迹

---

## ❓ 常见问题

### Q1: 训练很慢怎么办？

**A**: 
- ✅ 使用 GPU/MPS 加速（PyTorch 自动检测）
- ✅ 降低 `n_steps`（如 1024）减少内存占用
- ✅ 关闭不必要的后台程序
- ✅ 使用 DIRECT 模式而非 GUI 模式训练

### Q2: 成功率不高怎么办？

**A**:
1. **增加训练步数** → 至少 150,000 步
2. **检查奖励函数** → 确保鼓励正确行为
3. **调整学习率** → 尝试 1e-4 或 5e-4
4. **增加探索** → 提高 `ent_coef` 到 0.02

### Q3: 模型文件在哪？

**A**:
```
项目目录/
├── ppo_arm_final.zip      ← 最终模型（必需）
├── vec_normalize.pkl      ← 归一化参数（必需）
└── checkpoints/
    └── ppo_arm_*_steps.zip ← 中间检查点
```

### Q4: GUI 窗口打不开？

**A**:
- macOS: 确保允许终端访问屏幕录制
- Linux: 安装 `sudo apt-get install python3-opengl`
- 无头服务器: 使用 `--train` 模式（自动 DIRECT）

### Q5: 导入错误怎么办？

**A**:
```bash
# 重新安装所有依赖
pip uninstall pybullet stable-baselines3 gymnasium -y
pip install pybullet stable-baselines3 gymnasium
```

### Q6: 如何加速测试演示？

**A**: 修改 `rl_train.py` 第 234 和 479 行：

```python
# 更快
time.sleep(1/240)  # 或 time.sleep(0.01)

# 更慢（当前）
time.sleep(1/60)   # 或 time.sleep(0.03)
```

### Q7: 能在 Windows 上运行吗？

**A**: 
✅ 完全支持！只需注意：
- 使用 `python` 而非 `python3`
- 路径使用反斜杠 `\` 或正斜杠 `/`
- 激活环境: `venv\Scripts\activate`

---

## 🚀 进阶优化

### 自定义奖励函数

编辑 `rl_train.py` 的 `_compute_reward` 方法：

```python
def _compute_reward(self):
    info = {}
    distance = self._get_distance_to_target()
    
    # 🎯 自定义：距离奖励权重
    distance_reward = -distance * 15.0  # 增大到 15
    
    # 🎯 自定义：动作平滑惩罚
    if hasattr(self, 'prev_action'):
        action_change = np.sum(np.abs(self.current_action - self.prev_action))
        action_penalty = -action_change * 5.0
    else:
        action_penalty = 0.0
    
    # ... 其他奖励计算
    
    reward = distance_reward + progress_reward + contact_reward + action_penalty
    return reward, info
```

### 调整 PPO 参数

```python
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,        # 🔧 学习率
    n_steps=2048,              # 🔧 每次更新的步数
    batch_size=64,             # 🔧 小批量大小
    n_epochs=10,               # 🔧 每次更新的训练轮数
    gamma=0.99,                # 🔧 折扣因子
    gae_lambda=0.95,           # 🔧 GAE 参数
    clip_range=0.2,            # 🔧 裁剪范围
    ent_coef=0.01,             # 🔧 熵系数（探索度）
    vf_coef=0.5,               # 🔧 价值函数系数
    max_grad_norm=0.5,         # 🔧 梯度裁剪
    verbose=1
)
```

### 网络架构自定义

```python
from stable_baselines3 import PPO
from torch import nn

# 自定义网络
policy_kwargs = dict(
    net_arch=[dict(pi=[256, 256], vf=[256, 256])],  # 两层 256 神经元
    activation_fn=nn.ReLU
)

model = PPO(
    "MlpPolicy",
    env,
    policy_kwargs=policy_kwargs,
    # ... 其他参数
)
```

### 多环境并行训练

```python
from stable_baselines3.common.vec_env import SubprocVecEnv

# 创建 4 个并行环境
def make_env():
    return ArmReachEnv(render_mode=None, max_steps=500)

env = SubprocVecEnv([make_env for _ in range(4)])
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# 训练速度提升约 3-4 倍
```

---

## 📁 文件结构

```
pybullt/
├── rl_train.py              # 主训练脚本（核心）
├── test2.py                 # 手动控制演示
├── test_model.sh            # 测试脚本
├── test_slow.py             # 慢速测试（可选）
├── RL_README.md             # 本文档
│
├── ppo_arm_final.zip        # 训练好的模型
├── vec_normalize.pkl        # 归一化参数
│
├── checkpoints/             # 训练检查点
│   ├── ppo_arm_10000_steps.zip
│   ├── ppo_arm_20000_steps.zip
│   └── ...
│
└── ppo_arm_tensorboard/     # TensorBoard 日志
    └── PPO_*/
        └── events.out.tfevents.*
```

### 核心文件说明

| 文件 | 大小 | 说明 |
|------|------|------|
| `rl_train.py` | ~20 KB | 包含环境定义、训练、测试全流程 |
| `ppo_arm_final.zip` | ~150 KB | 神经网络权重 |
| `vec_normalize.pkl` | ~5 KB | 观察/奖励归一化参数 |

---

## 🎓 相关资源

### 论文

- [Proximal Policy Optimization Algorithms](https://arxiv.org/abs/1707.06347) - Schulman et al., 2017
- [PyBullet, a Python module for physics simulation](https://pybullet.org/wordpress/)

### 文档

- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
- [Gymnasium Documentation](https://gymnasium.farama.org/)

### 教程

- [Deep RL Course by Hugging Face](https://huggingface.co/learn/deep-rl-course/)
- [PyBullet Examples](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet)

---

## 📝 更新日志

### v1.0.0 (2025-10-14)

- ✅ 初始发布
- ✅ PPO 训练流程
- ✅ GUI 可视化
- ✅ 慢速演示模式
- ✅ 自动检查点保存
- ✅ TensorBoard 集成
- ✅ 完整文档

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

### 开发计划

- [ ] 添加更多机械臂模型（4-DoF, 6-DoF）
- [ ] 实现障碍物避让
- [ ] 支持多目标任务
- [ ] 添加摄像头视觉输入
- [ ] 集成真实机械臂硬件接口

---

## 📄 许可证

本项目仅供学习和研究使用。

---

## 💡 致谢

- PyBullet 团队提供优秀的物理引擎
- Stable-Baselines3 团队的 RL 框架
- OpenAI 的 Gymnasium 标准接口

---

<div align="center">

**⭐ 如果这个项目对你有帮助，请给个 Star！**

Made with ❤️ by [Your Name]

</div>
