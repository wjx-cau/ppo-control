# 强化学习机械臂训练

使用 PPO 算法训练机械臂末端接触绿色叶片。

## 安装依赖

```bash
conda activate bullet312
pip install stable-baselines3 gymnasium tensorboard
```

## 使用方法

### 1. 训练模型

```bash
python rl_train.py --train --timesteps 200000
```

训练参数：
- `--timesteps`: 训练总步数（默认 200000）
- 模型会保存到 `ppo_arm_final.zip`
- 中间检查点保存在 `./checkpoints/` 目录

### 2. 测试模型

```bash
python rl_train.py --test --episodes 5
```

测试参数：
- `--episodes`: 测试回合数（默认 5）
- `--model-path`: 指定模型路径（默认 `ppo_arm_final`）

### 3. 监控训练（可选）

使用 TensorBoard 查看训练曲线：

```bash
tensorboard --logdir ./ppo_arm_tensorboard/
```

## 环境说明

### 观察空间（13维）
- 3 个关节角度（rad）
- 3 个关节角速度（rad/s）
- 3 个末端位置坐标（x, y, z）
- 3 个目标位置坐标（x, y, z）
- 1 个末端到目标的距离

### 动作空间（3维）
- 3 个关节的角度增量（-0.1 ~ 0.1 rad）

### 奖励函数
- **距离奖励**：-distance × 10（越近越好）
- **进步奖励**：(prev_dist - curr_dist) × 50（鼓励靠近）
- **到达奖励**：
  - 距离 < 5cm：+50
  - 距离 < 2cm：+100
- **接触奖励**：末端与叶片接触时 +100
- **成功条件**：末端执行器接触到绿色叶片

## 训练技巧

1. **初期训练**（5-10 万步）：
   - 智能体学习基本的运动控制
   - 逐渐接近目标区域

2. **中期训练**（10-20 万步）：
   - 学习精确控制
   - 提高成功率

3. **微调**（20 万步以上）：
   - 优化轨迹
   - 提升稳定性

## 预期结果

训练约 15-20 万步后，智能体应该能够：
- 成功率 > 70%
- 平均步数 < 200
- 末端精准接触叶片

## 故障排查

### 训练不收敛
- 增加训练步数
- 调整学习率（默认 3e-4）
- 修改奖励函数权重

### 动作抖动
- 增加动作平滑惩罚
- 降低 `positionGains`
- 减小动作空间范围

### 内存不足
- 减少 `n_steps`（默认 2048）
- 使用更小的 `batch_size`（默认 64）
