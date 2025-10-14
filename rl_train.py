#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
强化学习训练脚本：使用 PPO 训练机械臂接触绿色叶片

用法：
  conda activate bullet312
  pip install stable-baselines3 gymnasium
  python rl_train.py --train  # 训练模式
  python rl_train.py --test   # 测试已训练模型
"""

import os
import time
import math
import tempfile
import argparse
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

# 复用 URDF 模板
URDF_TEMPLATE = """
<?xml version="1.0"?>
<robot name="three_dof_arm">
  <link name="base_link">
    <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0"/><geometry><box size="0.12 0.12 0.06"/></geometry><material name="grey"><color rgba="0.7 0.7 0.7 1"/></material></visual>
    <collision><origin xyz="0 0 0"/><geometry><box size="0.12 0.12 0.06"/></geometry></collision>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.03"/><axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="20" velocity="6"/>
    <dynamics damping="0.02" friction="0.0"/>
  </joint>
  <link name="link1">
    <inertial><origin xyz="0 0 0.11"/><mass value="0.4"/><inertia ixx="0.002" iyy="0.002" izz="0.002" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0.11"/><geometry><box size="0.04 0.04 0.22"/></geometry><material name="blue"><color rgba="0.2 0.4 0.9 1"/></material></visual>
    <collision><origin xyz="0 0 0.11"/><geometry><box size="0.04 0.04 0.22"/></geometry></collision>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.22"/><axis xyz="0 1 0"/>
    <limit lower="-1.745" upper="1.745" effort="20" velocity="6"/>
    <dynamics damping="0.02" friction="0.0"/>
  </joint>
  <link name="link2">
    <inertial><origin xyz="0.09 0 0"/><mass value="0.35"/><inertia ixx="0.002" iyy="0.002" izz="0.002" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0.09 0 0"/><geometry><box size="0.18 0.035 0.035"/></geometry><material name="orange"><color rgba="0.95 0.55 0.2 1"/></material></visual>
    <collision><origin xyz="0.09 0 0"/><geometry><box size="0.18 0.035 0.035"/></geometry></collision>
  </link>

  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0.18 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.094" upper="2.094" effort="20" velocity="6"/>
    <dynamics damping="0.02" friction="0.0"/>
  </joint>
  <link name="link3">
    <inertial><origin xyz="0.08 0 0"/><mass value="0.3"/><inertia ixx="0.002" iyy="0.002" izz="0.002" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0.08 0 0"/><geometry><box size="0.16 0.03 0.03"/></geometry><material name="green"><color rgba="0.2 0.8 0.4 1"/></material></visual>
    <collision><origin xyz="0.08 0 0"/><geometry><box size="0.16 0.03 0.03"/></geometry></collision>
  </link>

  <joint name="ee_fixed" type="fixed"><parent link="link3"/><child link="ee"/><origin xyz="0.16 0 0"/></joint>
  <link name="ee">
    <visual><origin xyz="0 0 0"/><geometry><box size="0.03 0.03 0.03"/></geometry><material name="red"><color rgba="0.9 0.2 0.2 1"/></material></visual>
    <collision><origin xyz="0 0 0"/><geometry><box size="0.03 0.03 0.03"/></geometry></collision>
  </link>
</robot>
"""


def write_urdf_tmp(text: str) -> str:
    """写入临时 URDF 文件"""
    tmpdir = tempfile.mkdtemp(prefix="arm3dof_rl_")
    urdf_path = os.path.join(tmpdir, "arm3dof.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(text)
    return urdf_path


class ArmReachEnv(gym.Env):
    """
    机械臂接触叶片的强化学习环境
    
    观察空间：
      - 3 个关节角度 (rad)
      - 3 个关节角速度 (rad/s)
      - 末端位置 (x, y, z)
      - 目标位置 (x, y, z)
      - 末端到目标的距离
    
    动作空间：
      - 3 个关节的目标角度增量 (连续动作)
    
    奖励：
      - 距离奖励：越接近目标奖励越高
      - 接触奖励：末端与叶片接触时给予大额奖励
      - 惩罚：过大的动作、超出关节限制
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}
    
    def __init__(self, render_mode=None, max_steps=500):
        super().__init__()
        
        self.render_mode = render_mode
        self.max_steps = max_steps
        self.current_step = 0
        
        # PyBullet 连接
        if render_mode == "human":
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240)
        
        # 加载场景
        self.plane_id = p.loadURDF("plane.urdf")
        urdf_path = write_urdf_tmp(URDF_TEMPLATE)
        self.arm_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True,
                                 flags=p.URDF_USE_SELF_COLLISION)
        
        # 叶片（目标）
        leaf_thickness = 0.004
        leaf_half_extents = [0.12, 0.08, leaf_thickness / 2]
        leaf_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=leaf_half_extents, 
                                       rgbaColor=[0.2, 0.7, 0.2, 1])
        leaf_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=leaf_half_extents)
        self.leaf_pos = [0.35, 0.0, 0.35]
        self.leaf_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=leaf_col, 
                                        baseVisualShapeIndex=leaf_vis,
                                        basePosition=self.leaf_pos, 
                                        baseOrientation=p.getQuaternionFromEuler([0, 0.0, 0]))
        
        # 关节信息
        self.control_joints = [0, 1, 2]
        
        # 找到末端执行器链接索引
        num_joints = p.getNumJoints(self.arm_id)
        self.ee_link_index = num_joints - 1  # 最后一个链接是末端执行器
        print(f"关节数: {num_joints}, 末端链接索引: {self.ee_link_index}")
        
        # 关节限制 (rad)
        self.joint_lower_limits = np.array([-3.14159, -1.745, -2.094])
        self.joint_upper_limits = np.array([3.14159, 1.745, 2.094])
        
        # 定义观察和动作空间
        # 观察：3 关节角 + 3 关节速度 + 3 末端位置 + 3 目标位置 + 1 距离 = 13
        obs_low = np.array(
            list(self.joint_lower_limits) + [-10]*3 +  # 关节角 + 速度
            [-1, -1, 0] + [-1, -1, 0] + [0],  # 末端位置 + 目标位置 + 距离
            dtype=np.float32
        )
        obs_high = np.array(
            list(self.joint_upper_limits) + [10]*3 +
            [1, 1, 1] + [1, 1, 1] + [2],
            dtype=np.float32
        )
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)
        
        # 动作：3 个关节的角度增量（-0.1 到 0.1 rad）
        self.action_space = spaces.Box(low=-0.1, high=0.1, shape=(3,), dtype=np.float32)
        
        # 初始化状态
        self.target_angles = np.zeros(3)
        self.prev_distance = None
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        self.current_step = 0
        
        # 随机初始关节角度
        if self.np_random is not None:
            init_angles = self.np_random.uniform(
                self.joint_lower_limits * 0.5,
                self.joint_upper_limits * 0.5
            )
        else:
            init_angles = np.zeros(3)
        
        # 重置关节状态
        for i, joint_idx in enumerate(self.control_joints):
            p.resetJointState(self.arm_id, joint_idx, init_angles[i])
        
        self.target_angles = init_angles.copy()
        
        # 执行几步让物理稳定
        for _ in range(10):
            p.stepSimulation()
        
        obs = self._get_obs()
        self.prev_distance = self._get_distance_to_target()
        
        return obs, {}
    
    def step(self, action):
        self.current_step += 1
        
        # 应用动作（增量）
        self.target_angles += action
        self.target_angles = np.clip(self.target_angles, 
                                     self.joint_lower_limits, 
                                     self.joint_upper_limits)
        
        # 发送关节控制命令
        p.setJointMotorControlArray(
            self.arm_id, 
            self.control_joints, 
            p.POSITION_CONTROL,
            targetPositions=self.target_angles,
            positionGains=[0.8]*3,
            velocityGains=[0.3]*3,
            forces=[50]*3
        )
        
        # 执行仿真步
        p.stepSimulation()
        if self.render_mode == "human":
            time.sleep(1/60)  # 降低到60Hz，让动作更清晰可见
        
        # 获取新状态
        obs = self._get_obs()
        
        # 计算奖励
        reward, info = self._compute_reward()
        
        # 终止条件
        terminated = info.get("success", False)
        truncated = self.current_step >= self.max_steps
        
        return obs, reward, terminated, truncated, info
    
    def _get_obs(self):
        """获取观察"""
        # 关节状态
        joint_states = [p.getJointState(self.arm_id, i) for i in self.control_joints]
        joint_angles = np.array([js[0] for js in joint_states], dtype=np.float32)
        joint_vels = np.array([js[1] for js in joint_states], dtype=np.float32)
        
        # 末端位置 - 使用 computeForwardKinematics 确保数据更新
        p.performCollisionDetection()
        ee_state = p.getLinkState(self.arm_id, self.ee_link_index, computeLinkVelocity=0)
        
        if ee_state is None or ee_state[0] is None:
            # 如果获取失败，使用默认位置
            ee_pos = np.array([0.0, 0.0, 0.5], dtype=np.float32)
        else:
            ee_pos = np.array(ee_state[0], dtype=np.float32)
        
        # 目标位置
        target_pos = np.array(self.leaf_pos, dtype=np.float32)
        
        # 距离
        distance = np.linalg.norm(ee_pos - target_pos)
        
        obs = np.concatenate([
            joint_angles,
            joint_vels,
            ee_pos,
            target_pos,
            [distance]
        ])
        
        return obs
    
    def _get_distance_to_target(self):
        """计算末端到目标的距离"""
        p.performCollisionDetection()
        ee_state = p.getLinkState(self.arm_id, self.ee_link_index, computeLinkVelocity=0)
        
        if ee_state is None or ee_state[0] is None:
            return 999.0  # 返回一个大值
        
        ee_pos = np.array(ee_state[0])
        target_pos = np.array(self.leaf_pos)
        return np.linalg.norm(ee_pos - target_pos)
    
    def _compute_reward(self):
        """计算奖励"""
        info = {}
        
        # 当前距离
        distance = self._get_distance_to_target()
        
        # 基础距离奖励：距离越小奖励越高
        distance_reward = -distance * 10.0
        
        # 进步奖励：如果比上一步更接近
        progress_reward = 0.0
        if self.prev_distance is not None:
            progress = self.prev_distance - distance
            progress_reward = progress * 50.0
        self.prev_distance = distance
        
        # 接触检测
        contact_points = p.getContactPoints(self.arm_id, self.leaf_id)
        contact_reward = 0.0
        success = False
        
        if len(contact_points) > 0:
            # 检查是否末端接触
            for cp in contact_points:
                if cp[3] == self.ee_link_index:  # 末端链接接触
                    contact_reward = 100.0
                    success = True
                    info["success"] = True
                    break
        
        # 到达奖励：距离小于阈值
        reach_reward = 0.0
        if distance < 0.05:
            reach_reward = 50.0
            if distance < 0.02:
                reach_reward = 100.0
        
        # 动作平滑惩罚（可选）
        action_penalty = 0.0
        
        # 总奖励
        reward = distance_reward + progress_reward + contact_reward + reach_reward + action_penalty
        
        info["distance"] = distance
        info["contact"] = len(contact_points) > 0
        
        return reward, info
    
    def close(self):
        p.disconnect(self.physics_client)


def train(args):
    """训练智能体"""
    print("=" * 60)
    print("开始训练强化学习智能体")
    print("=" * 60)
    
    # 创建环境
    def make_env():
        return ArmReachEnv(render_mode=None, max_steps=500)
    
    env = DummyVecEnv([make_env])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)
    
    # 创建模型
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        tensorboard_log="./ppo_arm_tensorboard/"
    )
    
    # 回调：定期保存
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path="./checkpoints/",
        name_prefix="ppo_arm"
    )
    
    # 训练
    total_timesteps = args.timesteps
    print(f"\n开始训练 {total_timesteps} 步...")
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_callback,
            progress_bar=True
        )
    except ImportError:
        # 如果没有安装 tqdm/rich，禁用进度条
        print("注意：进度条不可用，如需启用请安装: pip install tqdm rich")
        model.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_callback,
            progress_bar=False
        )
    
    # 保存最终模型
    model.save("ppo_arm_final")
    env.save("vec_normalize.pkl")
    print("\n训练完成！模型已保存到 ppo_arm_final.zip")


def test(args):
    """测试已训练的智能体"""
    print("=" * 60)
    print("测试已训练的智能体")
    print("=" * 60)
    
    # 加载模型
    model_path = args.model_path or "ppo_arm_final"
    print(f"加载模型：{model_path}")
    
    try:
        model = PPO.load(model_path)
        print("✓ 模型加载成功")
    except FileNotFoundError:
        print(f"✗ 模型文件未找到：{model_path}.zip")
        print("请先运行训练：python rl_train.py --train")
        return
    
    # 创建可视化环境
    env = ArmReachEnv(render_mode="human", max_steps=1000)
    
    # 如果有归一化参数，加载它
    try:
        env = VecNormalize.load("vec_normalize.pkl", DummyVecEnv([lambda: env]))
        env.training = False
        env.norm_reward = False
        print("✓ 归一化参数加载成功")
    except FileNotFoundError:
        print("⚠ 未找到归一化参数，使用未归一化的环境")
        pass
    
    # 运行测试回合
    num_episodes = args.episodes
    for episode in range(num_episodes):
        obs = env.reset()
        done = False
        total_reward = 0
        steps = 0
        text_id = None
        
        print(f"\n{'='*60}")
        print(f"第 {episode + 1}/{num_episodes} 回合")
        print(f"{'='*60}")
        print("👀 观察机械臂如何自主学习接触绿色叶片...\n")
        
        # 获取原始环境用于显示信息
        if isinstance(env, VecNormalize):
            raw_env = env.venv.envs[0]
        else:
            raw_env = env
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward
            steps += 1
            
            # 显示实时信息到GUI
            try:
                dist = raw_env._get_distance_to_target()
                status_text = f"步数: {steps} | 距离: {dist:.3f}m"
                
                # 移除旧的文字
                if text_id is not None:
                    try:
                        p.removeUserDebugItem(text_id)
                    except:
                        pass
                
                # 添加新的文字提示
                text_id = p.addUserDebugText(
                    status_text,
                    [0.3, 0.3, 0.6],
                    textColorRGB=[1, 1, 0],
                    textSize=1.8
                )
                
                # 每5步在控制台输出一次
                if steps % 5 == 0:
                    print(f"  步骤 {steps:3d}: 距离 {dist:.4f}m")
            except:
                pass
            
            # 额外延迟，让演示更慢更清晰
            time.sleep(0.03)  # 每步额外暂停30ms
            
            # 检查是否成功
            if isinstance(info, list):
                info = info[0]
            if info.get("success", False):
                # 清除状态文字
                if text_id is not None:
                    try:
                        p.removeUserDebugItem(text_id)
                    except:
                        pass
                
                # 显示成功消息
                try:
                    success_text_id = p.addUserDebugText(
                        "🎉 成功接触！",
                        [0.35, 0.0, 0.5],
                        textColorRGB=[0, 1, 0],
                        textSize=2.5
                    )
                except:
                    pass
                
                # 处理可能的 numpy 数组
                reward_val = float(total_reward.item()) if hasattr(total_reward, 'item') else float(total_reward)
                print(f"\n{'='*60}")
                print("🎉 成功！")
                print(f"{'='*60}")
                print(f"  步数: {steps}")
                print(f"  总奖励: {reward_val:.2f}")
                print(f"  末端成功接触到绿色叶片！")
                print(f"{'='*60}")
                
                time.sleep(2)  # 成功后暂停2秒欣赏
                
                try:
                    p.removeUserDebugItem(success_text_id)
                except:
                    pass
                break
        
        if not info.get("success", False):
            reward_val = float(total_reward.item()) if hasattr(total_reward, 'item') else float(total_reward)
            dist_val = info.get('distance', 'N/A')
            print(f"\n{'='*60}")
            print("❌ 未成功")
            print(f"{'='*60}")
            print(f"  步数: {steps}")
            print(f"  总奖励: {reward_val:.2f}")
            if dist_val != 'N/A':
                print(f"  最终距离: {dist_val:.3f}m")
            print(f"{'='*60}")
        
        # 清除所有文字
        if text_id is not None:
            try:
                p.removeUserDebugItem(text_id)
            except:
                pass
        
        # 回合间隔
        if episode < num_episodes - 1:
            print("\n⏳ 3秒后开始下一回合...")
            time.sleep(3)
    
    env.close()


def main():
    parser = argparse.ArgumentParser(description="强化学习训练机械臂接触叶片")
    parser.add_argument("--train", action="store_true", help="训练模式")
    parser.add_argument("--test", action="store_true", help="测试模式")
    parser.add_argument("--timesteps", type=int, default=200000, help="训练总步数")
    parser.add_argument("--episodes", type=int, default=5, help="测试回合数")
    parser.add_argument("--model-path", type=str, default=None, help="测试时加载的模型路径")
    
    args = parser.parse_args()
    
    if args.train:
        train(args)
    elif args.test:
        test(args)
    else:
        print("请指定模式：--train 或 --test")
        parser.print_help()


if __name__ == "__main__":
    main()
