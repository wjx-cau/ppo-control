# 关节映射调试

## URDF关节顺序（PyBullet索引）
```
索引0: lift_joint (prismatic) - 升降关节
索引1: joint1 (continuous) - J1底座旋转（Z轴）
索引2: joint2 (revolute) - J2肩部俯仰（Y轴）
索引3: joint3 (revolute) - J3肘部俯仰（Y轴）
```

## 代码中的映射
```python
lift_joint_idx = 0              # 升降关节
control_joints = [1, 2, 3]      # 旋转关节数组

rate[0] -> control_joints[0] -> joint1 (J1底座)
rate[1] -> control_joints[1] -> joint2 (J2肩部)
rate[2] -> control_joints[2] -> joint3 (J3肘部)

q_deg[0] -> control_joints[0] -> joint1 (J1底座)
q_deg[1] -> control_joints[1] -> joint2 (J2肩部)
q_deg[2] -> control_joints[2] -> joint3 (J3肘部)
```

## 按键控制映射
```
⬆️/⬇️    -> lift_height -> lift_joint (索引0)
J/L      -> rate[0]/q_deg[0] -> control_joints[0] -> joint1 (索引1, J1底座旋转)
I/K      -> rate[1]/q_deg[1] -> control_joints[1] -> joint2 (索引2, J2肩部)
U/O      -> rate[2]/q_deg[2] -> control_joints[2] -> joint3 (索引3, J3肘部)
Q/P      -> track_angle -> 约束位置更新（轨道旋转）
```

## 滑块映射
```
Slider 0 (Lift height) -> lift_height -> lift_joint (索引0)
Slider 1 (J1 yaw)      -> q_deg[0] -> control_joints[0] -> joint1 (索引1)
Slider 2 (J2 pitch)    -> q_deg[1] -> control_joints[1] -> joint2 (索引2)
Slider 3 (J3 pitch)    -> q_deg[2] -> control_joints[2] -> joint3 (索引3)
```

## 电机控制调用
```python
# 升降控制
p.setJointMotorControl2(arm_id, lift_joint_idx, ...)  # 控制索引0

# 旋转控制
p.setJointMotorControlArray(arm_id, control_joints, ..., targetPositions=q_rad)
# 相当于：
# control_joints[0]=1 -> q_rad[0] -> joint1
# control_joints[1]=2 -> q_rad[1] -> joint2  
# control_joints[2]=3 -> q_rad[2] -> joint3
```

## 验证步骤
1. 按H开启日志
2. 按J键，观察是否有 "J" 字符输出
3. 按T键，查看当前关节角度
4. 观察机械臂是否响应

## 可能的问题
- [x] 滑块覆盖键盘输入 - 已修复（改为单独检查每个滑块）
- [ ] 按键映射物理意义不对
- [ ] dt时间步太小导致积分效果不明显
- [ ] 角速度rate_unit值太小
