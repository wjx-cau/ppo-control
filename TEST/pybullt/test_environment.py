#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyBullet 3-DoF 机械臂 + 圆形  <link name="link2">
    <inertial><origin xyz="0.09 0 0"/><mass value="0.15"/><inertia ixx="0.002" iyy="0.002" izz="0.002" ixy="0" ixz="0" iyz="0"/></inertial>
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
    <inertial><origin xyz="0.08 0 0"/><mass value="0.12"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>法：
  conda activate bullet312
  python test_environment.py

操作：
  - 机械臂关节键盘控制（点击3D窗口确保焦点）：
      Y/H：升降 (0-0.5米，Y向上，H向下)
      I/K：J1 底座旋转（360度无限制）
      J/L：J2 肩部俯仰（±100°）
      U/O：J3 肘部俯仰（±120°）
    支持"按住连续变化"和"点按步进"(5°/次)。
  
  - 🆕 轨道底座控制：
      Q：逆时针旋转（沿轨道移动）
      P：顺时针旋转（沿轨道移动）
  
  - 🆕 末端摄像机：
      C：开启/关闭摄像机
      V：调节渲染频率（高/中/低）
  
  - 调试：
      G：开关按键日志
      T：打印当前状态

说明：
  - 机械臂基座安装在圆形轨道上，可以沿轨道旋转移动
  - 轨道半径：0.5 米
  - 键盘采用"角速度积分 + 点按步进"的双机制
  - 已删除滑块，纯键盘控制更流畅
  - 关节索引: [1]=lift_joint, [2]=J1(底座), [3]=J2(肩部), [4]=J3(肘部)
"""

import os
import time
import math
import tempfile
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="three_dof_arm">
  <link name="base_link">
    <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0"/><geometry><box size="0.12 0.12 0.06"/></geometry><material name="grey"><color rgba="0.7 0.7 0.7 1"/></material></visual>
    <collision><origin xyz="0 0 0"/><geometry><box size="0.12 0.12 0.06"/></geometry></collision>
  </link>

  <!-- 🆕 固定支撑杆 (fixed joint) - 从基座到升降杆底部 -->
  <joint name="support_fixed" type="fixed">
    <parent link="base_link"/><child link="support_rod"/>
    <origin xyz="0 0 0.03"/>
  </joint>
  <link name="support_rod">
    <inertial><origin xyz="0 0 0.25"/><mass value="0.5"/><inertia ixx="0.01" iyy="0.01" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <!-- 实体支撑圆柱：半径0.025m，高度0.5m -->
    <visual><origin xyz="0 0 0.25"/><geometry><cylinder radius="0.025" length="0.5"/></geometry><material name="dark_silver"><color rgba="0.6 0.6 0.65 1"/></material></visual>
    <collision><origin xyz="0 0 0.25"/><geometry><cylinder radius="0.025" length="0.5"/></geometry></collision>
  </link>

  <!-- 升降关节 (prismatic) - 移动平台沿支撑杆滑动 -->
  <joint name="lift_joint" type="prismatic">
    <parent link="support_rod"/><child link="lift_link"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0.0" upper="0.5" effort="5000" velocity="3.0"/>
    <dynamics damping="0.8" friction="0.0"/>
  </joint>
  <link name="lift_link">
    <inertial><origin xyz="0 0 0"/><mass value="0.1"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <!-- 升降平台（圆盘套在支撑杆上）-->
    <visual><origin xyz="0 0 0"/><geometry><cylinder radius="0.06" length="0.03"/></geometry><material name="silver"><color rgba="0.8 0.8 0.85 1"/></material></visual>
    <collision><origin xyz="0 0 0"/><geometry><cylinder radius="0.06" length="0.03"/></geometry></collision>
  </link>

  <joint name="joint1" type="continuous">
    <parent link="lift_link"/><child link="link1"/>
    <origin xyz="0 0 0.015"/><axis xyz="0 0 1"/>
    <limit effort="20" velocity="6"/>
    <dynamics damping="0.02" friction="0.0"/>
  </joint>
  <link name="link1">
    <inertial><origin xyz="0 0 0.11"/><mass value="0.2"/><inertia ixx="0.002" iyy="0.002" izz="0.002" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0.11"/><geometry><box size="0.04 0.04 0.22"/></geometry><material name="blue"><color rgba="0.2 0.4 0.9 1"/></material></visual>
    <collision><origin xyz="0 0 0.11"/><geometry><box size="0.04 0.04 0.22"/></geometry></collision>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.22"/><axis xyz="0 1 0"/>
    <limit lower="-1.745" upper="1.745" effort="20" velocity="6"/>
    <dynamics damping="0.5" friction="0.0"/>
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
    <dynamics damping="0.5" friction="0.0"/>
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
    tmpdir = tempfile.mkdtemp(prefix="arm3dof_")
    urdf_path = os.path.join(tmpdir, "arm3dof.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(text)
    return urdf_path


def main() -> None:
    try:
        p.connect(p.GUI)
    except Exception:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=40, cameraPitch=-35,
                                 cameraTargetPosition=[0.0, 0.0, 0.15])
    try:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)  # 🔑 关闭默认快捷键，释放方向键
    except Exception:
        pass

    p.loadURDF("plane.urdf")

    # ====== 创建圆形轨道底座 ======
    track_radius = 0.5  # 轨道半径 0.5 米
    track_width = 0.08
    track_height = 0.05
    
    # 轨道环（可视化）
    track_visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=track_radius,
        length=track_height,
        rgbaColor=[0.3, 0.3, 0.3, 0.6]
    )
    track_collision = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=track_radius,
        height=track_height
    )
    track_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=track_collision,
        baseVisualShapeIndex=track_visual,
        basePosition=[0, 0, track_height/2],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
    )
    
    # 轨道内圈（遮挡，营造"环形"效果）
    inner_radius = track_radius - track_width
    inner_visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=inner_radius,
        length=track_height + 0.001,
        rgbaColor=[0.5, 0.5, 0.5, 1.0]
    )
    inner_collision = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=inner_radius,
        height=track_height + 0.001
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=inner_collision,
        baseVisualShapeIndex=inner_visual,
        basePosition=[0, 0, track_height/2],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
    )
    
    # 轨道位置角度（弧度）
    track_angle = 0.0  # 机械臂在轨道上的角度位置
    
    # 计算机械臂基座初始位置
    arm_x = track_radius * math.cos(track_angle)
    arm_y = track_radius * math.sin(track_angle)
    arm_z = track_height  # 放在轨道上方
    
    # ====== 加载机械臂（放在轨道上） ======
    urdf_path = write_urdf_tmp(URDF_TEMPLATE)
    arm_id = p.loadURDF(
        urdf_path, 
        basePosition=[arm_x, arm_y, arm_z], 
        baseOrientation=p.getQuaternionFromEuler([0, 0, track_angle]),
        useFixedBase=False,  # 非固定，但通过约束固定
        flags=p.URDF_USE_SELF_COLLISION
    )
    assert isinstance(arm_id, int), "arm_id not set"
    print(f"arm_id={arm_id}")
    
    # 增加基座质量，防止漂移
    p.changeDynamics(arm_id, -1, mass=5, linearDamping=0.5, angularDamping=0.5)
    
    # 创建固定约束（将机械臂基座固定在当前位置）
    # 这样可以用changeConstraint动态改变位置
    constraint_id = p.createConstraint(
        parentBodyUniqueId=arm_id,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[arm_x, arm_y, arm_z],
        childFrameOrientation=p.getQuaternionFromEuler([0, 0, track_angle])
    )
    # 设置约束的最大力，确保约束足够强
    p.changeConstraint(constraint_id, maxForce=10000)
    print(f"constraint_id={constraint_id}")

    num_joints = p.getNumJoints(arm_id)
    print(f"num_joints={num_joints}")
    
    # 🔍 打印关节信息表，确认索引
    print("\n关节信息表:")
    for i in range(num_joints):
        ji = p.getJointInfo(arm_id, i)
        print(f"  [{i}] {ji[1].decode():20s} 类型={ji[2]} (0=REVOLUTE, 1=PRISMATIC, 4=FIXED)")
    print()
    
    # 🔧 关节索引（根据实际URDF结构）：
    # [0]=support_fixed(FIXED), [1]=lift_joint(PRISMATIC), [2]=joint1(REVOLUTE), [3]=joint2, [4]=joint3, [5]=ee_fixed
    lift_joint_idx = 1  # ✓ lift_joint 在索引 1
    control_joints = [2, 3, 4]  # ✓ 旋转关节：J1=2, J2=3, J3=4
    
    # 🆕 已移除四根调试线，现在使用URDF中的实体支撑杆

    # 坐标轴辅助（固定在世界坐标）
    p.addUserDebugLine([0, 0, 0], [0.3, 0, 0], [1, 0, 0], lineWidth=3)
    p.addUserDebugLine([0, 0, 0], [0, 0.3, 0], [0, 1, 0], lineWidth=3)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.3], [0, 0, 1], lineWidth=3)

    # ====== 创建植株（茎干 + 多片叶子 + 叶茎）======
    plant_base_pos = [0.0, 0.0, 0.0]  # 植株基座位置（轨道中心）
    plant_object_ids = []  # 保存所有植株物体ID，用于设置物理属性
    
    # 1. 土壤基座（棕色圆柱）
    soil_radius = 0.15
    soil_height = 0.08
    soil_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=soil_radius, length=soil_height, 
                                   rgbaColor=[0.4, 0.25, 0.1, 1])
    soil_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=soil_radius, height=soil_height)
    soil_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=soil_col, baseVisualShapeIndex=soil_vis,
                      basePosition=[plant_base_pos[0], plant_base_pos[1], soil_height/2])
    plant_object_ids.append(soil_id)
    
    # 2. 主茎干（棕色圆柱）- 更高更粗
    stem_radius = 0.018
    stem_height = 0.65  # 增高到0.65米，与机械臂总高度接近
    stem_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=stem_radius, length=stem_height,
                                   rgbaColor=[0.4, 0.25, 0.1, 1])  # 棕色
    stem_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=stem_radius, height=stem_height)
    stem_base_z = soil_height
    stem_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=stem_col, baseVisualShapeIndex=stem_vis,
                      basePosition=[plant_base_pos[0], plant_base_pos[1], stem_base_z + stem_height/2])
    plant_object_ids.append(stem_id)
    
    # 3. 多片叶子（椭圆形，带叶茎，不同高度和角度）
    leaf_thickness = 0.005
    leaf_stem_radius = 0.005  # 叶茎半径
    # 均匀分布7片叶子，避免交叉
    leaf_num = 7
    leaf_configs = []
    for i in range(leaf_num):
        height = 0.18 + i * 0.07  # 层高递增
        angle_deg = (360 / leaf_num) * i  # 均匀分布
        scale = 1.0 - i * 0.03  # 越高越小
        leaf_configs.append((height, angle_deg, scale))
    
    for height, angle_deg, scale in leaf_configs:
        # 计算叶子位置（围绕茎干螺旋分布）
        angle_rad = math.radians(angle_deg)
        leaf_distance = 0.12 * scale  # 叶子离主茎的距离
        leaf_stem_length = leaf_distance * 0.7  # 叶茎长度
        
        # 叶茎起点（连接到主茎）
        stem_start = [
            plant_base_pos[0],
            plant_base_pos[1],
            stem_base_z + height
        ]
        
        # 叶茎方向
        leaf_stem_dir_x = math.cos(angle_rad)
        leaf_stem_dir_y = math.sin(angle_rad)
        
        # 叶茎中点位置
        leaf_stem_pos = [
            stem_start[0] + leaf_stem_dir_x * leaf_stem_length / 2,
            stem_start[1] + leaf_stem_dir_y * leaf_stem_length / 2,
            stem_start[2]
        ]
        
        # 创建叶茎（细圆柱，棕绿色）
        leaf_stem_vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=leaf_stem_radius,
            length=leaf_stem_length,
            rgbaColor=[0.3, 0.6, 0.2, 1]  # 棕绿色
        )
        leaf_stem_col = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=leaf_stem_radius,
            height=leaf_stem_length
        )
        
        # 叶茎朝向（水平伸出，稍微向上）
        leaf_stem_orientation = p.getQuaternionFromEuler([
            0,
            math.radians(75),  # 向水平方向倾斜
            angle_rad
        ])
        
        leaf_stem_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=leaf_stem_col,
            baseVisualShapeIndex=leaf_stem_vis,
            basePosition=leaf_stem_pos,
            baseOrientation=leaf_stem_orientation
        )
        plant_object_ids.append(leaf_stem_id)
        
        # 叶子形状（单一平面，正面深绿色）
        leaf_size = [0.11 * scale, 0.08 * scale, leaf_thickness/2]
        leaf_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=leaf_size,
            rgbaColor=[0.2, 0.8, 0.3, 1]  # 正面深绿色
        )
        leaf_col = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=leaf_size
        )
        # 叶子位置（在叶茎末端）
        leaf_pos = [
            stem_start[0] + leaf_stem_dir_x * leaf_distance,
            stem_start[1] + leaf_stem_dir_y * leaf_distance,
            stem_start[2]
        ]
        # 叶子朝向（统一向上倾斜20°，不再绕z轴旋转）
        leaf_orientation = p.getQuaternionFromEuler([
            math.radians(20),  # 向上倾斜20度
            0,
            0
        ])
        leaf_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=leaf_col,
            baseVisualShapeIndex=leaf_vis,
            basePosition=leaf_pos,
            baseOrientation=leaf_orientation
        )
        plant_object_ids.append(leaf_id)
    
    # 4. 顶部花朵/目标点（红色小球）
    flower_radius = 0.025
    flower_vis = p.createVisualShape(p.GEOM_SPHERE, radius=flower_radius,
                                     rgbaColor=[0.9, 0.2, 0.2, 1])  # 红色
    flower_col = p.createCollisionShape(p.GEOM_SPHERE, radius=flower_radius)
    flower_height = stem_base_z + stem_height + 0.03
    flower_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=flower_col, baseVisualShapeIndex=flower_vis,
                      basePosition=[plant_base_pos[0], plant_base_pos[1], flower_height])
    plant_object_ids.append(flower_id)
    
    # 设置植株物理属性（柔软，不弹开机械臂）
    for obj_id in plant_object_ids:
        p.changeDynamics(obj_id, -1,
                        restitution=0.0,           # 无弹性
                        lateralFriction=0.5,       # 摩擦力
                        rollingFriction=0.01,      # 滚动摩擦
                        spinningFriction=0.01,     # 旋转摩擦
                        contactStiffness=1000,     # 低接触刚度（柔软）
                        contactDamping=50)         # 接触阻尼

    # 设置关节的物理属性（不包括基座-1）
    for link in range(0, num_joints):
        try:
            p.changeDynamics(arm_id, link, 
                           linearDamping=0.1,    # 轻微线性阻尼防止抖动
                           angularDamping=0.1,   # 轻微角阻尼防止抖动
                           restitution=0.0,      # 无弹性，防止反弹
                           contactStiffness=10000,  # 高接触刚度
                           contactDamping=100)      # 接触阻尼
        except Exception:
            pass

    # 演示 5 秒（机械臂动作 + 轨道移动 + 升降 + J1 旋转 180°）
    print("演示模式：机械臂挥舞 + 轨道旋转 + 升降 + J1 扫描...")
    print("🎯 J1 将旋转 180° 扫描环境")
    t0 = time.time()
    demo_duration = 5.0
    while time.time() - t0 < demo_duration:
        t = time.time() - t0
        progress = t / demo_duration  # 0 到 1
        
        lift_pos = 0.2 + 0.15 * math.sin(1.2 * t)  # 升降演示
        # J1 从 -90° 旋转到 +90° (共 180°)
        q1 = math.radians(-90 + 180 * progress)  # -90° → +90°
        q2 = math.radians(30.0 + 15.0 * math.sin(1.7 * t))
        q3 = math.radians(40.0 * math.sin(2.0 * t))
        
        # 控制升降关节
        p.setJointMotorControl2(arm_id, lift_joint_idx, p.POSITION_CONTROL,
                               targetPosition=lift_pos, positionGain=0.8, force=600)
        # 控制旋转关节
        p.setJointMotorControlArray(arm_id, control_joints, p.POSITION_CONTROL,
                                    targetPositions=[q1, q2, q3], positionGains=[0.45]*3, forces=[40]*3)
        
        # 演示时轨道也缓慢旋转 - 使用约束更新
        track_angle += 0.3 * (1/240)  # 约 17°/秒
        arm_x = track_radius * math.cos(track_angle)
        arm_y = track_radius * math.sin(track_angle)
        p.changeConstraint(
            constraint_id,
            jointChildPivot=[arm_x, arm_y, arm_z],
            jointChildFrameOrientation=p.getQuaternionFromEuler([0, 0, track_angle]),
            maxForce=10000
        )
        
        p.stepSimulation()
        time.sleep(1/240.0)  # 保持 240Hz 仿真频率

    # 🆕 演示结束后，调整到朝向植物的初始姿态
    print("\n🎯 调整初始姿态：让摄像头朝向植物...")
    
    # 读取演示结束后的当前状态
    current_lift_state = p.getJointState(arm_id, lift_joint_idx)[0]
    current_joints = [p.getJointState(arm_id, j)[0] for j in control_joints]
    
    # 目标姿态：
    # - 轨道角度：0° (正对植物)
    # - 升降高度：0.3m (与果实高度0.76m接近)
    # - J1: 0° (手臂正对植物)
    # - J2: 20° (稍微向上)
    # - J3: -20° (肘部稍微弯曲)
    
    target_track_angle = 0.0
    target_lift = 0.3
    target_q = [math.radians(0), math.radians(20), math.radians(-20)]
    
    # 平滑过渡到目标姿态（1.5秒）
    transition_time = 1.5
    t0 = time.time()
    
    while time.time() - t0 < transition_time:
        t = (time.time() - t0) / transition_time  # 0 到 1
        
        # 插值
        current_track = track_angle + (target_track_angle - track_angle) * t
        current_lift = current_lift_state + (target_lift - current_lift_state) * t
        interp_q = [current_joints[i] + (target_q[i] - current_joints[i]) * t for i in range(3)]
        
        # 更新轨道位置
        arm_x = track_radius * math.cos(current_track)
        arm_y = track_radius * math.sin(current_track)
        p.changeConstraint(
            constraint_id,
            jointChildPivot=[arm_x, arm_y, arm_z],
            jointChildFrameOrientation=p.getQuaternionFromEuler([0, 0, current_track]),
            maxForce=10000
        )
        
        # 控制升降和关节
        p.setJointMotorControl2(arm_id, lift_joint_idx, p.POSITION_CONTROL,
                               targetPosition=current_lift, positionGain=0.8, force=600)
        p.setJointMotorControlArray(arm_id, control_joints, p.POSITION_CONTROL,
                                    targetPositions=interp_q, positionGains=[0.45]*3, forces=[40]*3)
        
        p.stepSimulation()
        time.sleep(1/240.0)
    
    # 更新全局变量到目标值
    track_angle = target_track_angle
    lift_height = target_lift
    q_deg = [math.degrees(q) for q in target_q]
    
    print(f"✅ 初始姿态已就位：轨道={math.degrees(track_angle):.1f}°, "
          f"升降={lift_height:.2f}m, J1={q_deg[0]:.1f}°")
    print("   摄像头现在应该能看到植物了！\n")

    # 🗑️ 已删除滑块 - 只使用键盘控制

    # 实时仿真，提升交互
    try:
        p.setRealTimeSimulation(0)
    except Exception:
        pass

    print("Switch to interactive control.")
    print("[Hint] 升降控制: Y/H (Y向上, H向下)")
    print("[Hint] 机械臂关节: I/K→J1底座旋转, J/L→J2肩部, U/O→J3肘部")
    print("[Hint] 🆕 轨道移动: Q=逆时针, P=顺时针")
    print("[Hint] 🆕 末端摄像机: 按C键开启/关闭 | 按V键调节渲染频率")
    print("[Hint] Debug: press 'G' to toggle key logging, 'T' to print track angle")
    print("[Hint] 🤖 外部控制: 支持通过命令文件控制（qwen-control.py）")

    # 🔧 读取演示结束后的实际关节位置作为初始值
    lift_height = p.getJointState(arm_id, lift_joint_idx)[0]  # 获取当前升降高度
    current_joints = [p.getJointState(arm_id, j)[0] for j in control_joints]
    q_deg = [math.degrees(q) for q in current_joints]  # 转换为角度
    print(f"[Init] lift_height={lift_height:.3f}m, q_deg={[round(q,1) for q in q_deg]}")

    # 🤖 外部命令接口（通过文件通信）
    command_file = "/tmp/pybullet_arm_command.txt"
    status_file = "/tmp/pybullet_arm_status.txt"
    state_file = "/tmp/pybullet_arm_state.json"  # 🆕 详细状态信息
    
    # 创建状态文件，标记模拟器已就绪
    with open(status_file, "w", encoding="utf-8") as f:
        f.write("READY\n")
    print(f"[External Control] 状态文件已创建: {status_file}")
    
    # 外部按键缓冲区（模拟按键按下）
    external_keys = {}  # {key_char: (start_time, duration)}
    
    def read_external_commands():
        """读取外部命令文件"""
        nonlocal external_keys
        try:
            if os.path.exists(command_file):
                with open(command_file, "r", encoding="utf-8") as f:
                    lines = f.readlines()
                
                if lines:
                    # 解析命令：格式为 "KEY DURATION" (如 "Q 2.0")
                    for line in lines:
                        line = line.strip()
                        if not line or line.startswith("#"):
                            continue
                        
                        parts = line.split()
                        if len(parts) >= 2:
                            key = parts[0].upper()
                            duration = float(parts[1])
                            external_keys[key] = (time.time(), duration)
                            print(f"[External] 🎮 接收命令: {key} 持续 {duration:.2f}s")
                    
                    # 清空命令文件
                    os.remove(command_file)
        except Exception as e:
            # 打印异常以便调试
            if os.path.exists(command_file):
                print(f"[External] ⚠️ 读取命令失败: {e}")
    
    def check_external_key(key_char):
        """检查外部按键是否被按下（模拟 KEY_IS_DOWN）"""
        if key_char in external_keys:
            start_time, duration = external_keys[key_char]
            elapsed = time.time() - start_time
            
            if elapsed < duration:
                print(f"[External] ✓ 按键 {key_char} 仍按下中 ({elapsed:.3f}/{duration:.3f}s)")
                return True  # 仍在按下状态
            else:
                # 按键时间结束，移除
                del external_keys[key_char]
                print(f"[External] ✗ 按键 {key_char} 已释放")
                return False
        return False

    # 键盘速度与步进
    rate_unit = 120.0           # 按住时的角速度（deg/s）
    tap_step = 5.0              # 点按一次的步进（度）
    lift_rate_unit = 0.5        # 升降速度（m/s）- 提高到0.5，原来0.2太慢
    lift_tap_step = 0.05        # 升降点按步进（米，即5cm）- 增大步进
    lift_rate = 0.0             # 当前升降速度（持久变量，支持平滑过渡）
    
    # j1 无限位（设置为很大的范围），j2/j3 有实际限位
    limits_lo = [-99999.0, -100.0, -120.0]
    limits_hi = [ 99999.0,  100.0,  120.0]
    lift_limits = [0.0, 0.5]  # 升降范围：0-0.5米（匹配URDF）
    
    # 轨道旋转速度（rad/s）
    track_rotation_speed = 0.5  # 约 28.6°/秒
    track_tap_step = math.radians(10)  # 点按一次旋转 10°

    # 调试开关
    key_log = False
    
    # 末端摄像机开关（按C键切换）
    camera_enabled = False
    camera_window_id = None  # 用于存储摄像机窗口ID
    camera_frame_skip = 10   # 每10帧渲染一次摄像机（降低频率提升性能）
    camera_frame_counter = 0
    
    # 🔧 升降位置同步计数器（避免累积误差）
    lift_sync_counter = 0
    lift_sync_interval = 120  # 每120帧（约0.5秒）同步一次，减少干扰

    # 工具：检测按住/点按（ASCII 与特殊键）- 支持外部按键
    def _down_ascii(keys, ch):
        # 先检查外部按键
        if check_external_key(ch.upper()):
            return True
        # 再检查物理键盘
        code1, code2 = ord(ch.lower()), ord(ch.upper())
        v1, v2 = keys.get(code1, 0), keys.get(code2, 0)
        return (v1 & p.KEY_IS_DOWN) or (v2 & p.KEY_IS_DOWN)

    def _hit_ascii(keys, ch):
        code1, code2 = ord(ch.lower()), ord(ch.upper())
        v1, v2 = keys.get(code1, 0), keys.get(code2, 0)
        return (v1 & p.KEY_WAS_TRIGGERED) or (v2 & p.KEY_WAS_TRIGGERED)

    def _down_code(keys, code):
        v = keys.get(code, 0)
        return (v & p.KEY_IS_DOWN)

    def _hit_code(keys, code):
        v = keys.get(code, 0)
        return (v & p.KEY_WAS_TRIGGERED)

    prev_t = time.time()

    while p.isConnected():
        now = time.time()
        dt = max(1e-3, min(0.05, now - prev_t))
        prev_t = now

        # 🗑️ 已删除滑块检查代码
        
        # 🤖 读取外部命令（每帧检查一次）
        read_external_commands()

        keys = p.getKeyboardEvents()

        # 调试：按 G 开关日志，按 T 打印轨道角度
        if _hit_ascii(keys, 'g'):
            key_log = not key_log
            print("[KeyLog]", "ON" if key_log else "OFF")
        if _hit_ascii(keys, 't'):
            js = [p.getJointState(arm_id, i)[0] for i in control_joints]
            print(f"[Track angle] {math.degrees(track_angle):.1f}°")
            print(f"[Joints rad] {[round(x,3) for x in js]}")
        
        # 🆕 摄像机切换（C键）- 支持外部命令
        if _hit_ascii(keys, 'c') or check_external_key('C'):
            camera_enabled = not camera_enabled
            if camera_enabled:
                print(f"📷 [Camera] 末端摄像机已开启 (每{camera_frame_skip}帧渲染一次)")
            else:
                print("📷 [Camera] 末端摄像机已关闭")
        
        # 🆕 调节摄像机渲染频率（V键）
        if _hit_ascii(keys, 'v'):
            if camera_frame_skip == 10:
                camera_frame_skip = 5  # 更流畅
                print("📷 [Camera] 渲染频率: 高 (每5帧)")
            elif camera_frame_skip == 5:
                camera_frame_skip = 20  # 更省性能
                print("📷 [Camera] 渲染频率: 低 (每20帧)")
            else:
                camera_frame_skip = 10  # 默认
                print("📷 [Camera] 渲染频率: 中 (每10帧)")

        rate = [0.0, 0.0, 0.0]
        track_rate = 0.0  # 轨道旋转速度
        
        # ====== 升降控制（Y/H键）- 平滑加减速 ======
        target_lift_rate = 0.0  # 目标速度
        
        # Y: 向上升降
        if _down_ascii(keys, 'y'):
            target_lift_rate = lift_rate_unit
            if key_log and lift_rate == 0.0: print(f"Y 按住: target={target_lift_rate}")
        if _hit_ascii(keys, 'y'):
            lift_height += lift_tap_step
            if key_log: print(f"Y 点击: lift_height={lift_height}")
        
        # H: 向下升降
        if _down_ascii(keys, 'h'):
            target_lift_rate = -lift_rate_unit
            if key_log and lift_rate == 0.0: print(f"H 按住: target={target_lift_rate}")
        if _hit_ascii(keys, 'h'):
            lift_height -= lift_tap_step
            if key_log: print(f"H 点击: lift_height={lift_height}")
        
        # 平滑过渡到目标速度（加减速时间约0.2秒，更平滑）
        acceleration = 2.5  # m/s² (0.5m/s / 0.2s) - 降低加速度，减少冲击
        if abs(target_lift_rate - lift_rate) > 0.01:
            if target_lift_rate > lift_rate:
                lift_rate = min(lift_rate + acceleration * dt, target_lift_rate)
            else:
                lift_rate = max(lift_rate - acceleration * dt, target_lift_rate)
        else:
            lift_rate = target_lift_rate
        
        # 升降积分 + 限位
        lift_height += lift_rate * dt
        lift_height = max(lift_limits[0], min(lift_limits[1], lift_height))
        
        # 🔧 定期同步实际位置，避免累积误差
        lift_sync_counter += 1
        if lift_sync_counter >= lift_sync_interval:
            lift_sync_counter = 0
            # 只在静止时同步，避免干扰运动
            if lift_rate == 0.0:
                actual_lift = p.getJointState(arm_id, lift_joint_idx)[0]
                error = abs(actual_lift - lift_height)
                
                # 只有误差较大时才同步（5mm阈值）
                if error > 0.005:
                    if key_log: print(f"🔄 同步: lift_height {lift_height:.3f} → {actual_lift:.3f} (误差{error*1000:.1f}mm)")
                    lift_height = actual_lift

        # ====== 轨道控制（Q/P键）======
        # Q：逆时针旋转（增加角度）
        if _down_ascii(keys, 'q'):
            track_rate += track_rotation_speed
        if _hit_ascii(keys, 'q'):
            track_angle += track_tap_step
        
        # P：顺时针旋转（减少角度）
        if _down_ascii(keys, 'p'):
            track_rate -= track_rotation_speed
        if _hit_ascii(keys, 'p'):
            track_angle -= track_tap_step

        # ===== 关节键位：I/K -> J1，J/L -> J2，U/O -> J3 =====
        # J1 (底座旋转, control_joints[0]=joint1)
        if _down_ascii(keys, 'i'):
            rate[0] += rate_unit
            if key_log: print(f"I 按住: rate[0]={rate[0]} (J1)")
        if _down_ascii(keys, 'k'):
            rate[0] -= rate_unit
            if key_log: print(f"K 按住: rate[0]={rate[0]} (J1)")
        if _hit_ascii(keys, 'i'):
            q_deg[0] += tap_step
            if key_log: print(f"I 点击: q_deg[0]={q_deg[0]} (J1)")
        if _hit_ascii(keys, 'k'):
            q_deg[0] -= tap_step
            if key_log: print(f"K 点击: q_deg[0]={q_deg[0]} (J1)")

        # J2 (肩部俯仰, control_joints[1]=joint2)
        if _down_ascii(keys, 'j'):
            rate[1] += rate_unit
            if key_log: print(f"J 按住: rate[1]={rate[1]} (J2)")
        if _down_ascii(keys, 'l'):
            rate[1] -= rate_unit
            if key_log: print(f"L 按住: rate[1]={rate[1]} (J2)")
        if _hit_ascii(keys, 'j'):
            q_deg[1] += tap_step
            if key_log: print(f"J 点击: q_deg[1]={q_deg[1]} (J2)")
        if _hit_ascii(keys, 'l'):
            q_deg[1] -= tap_step
            if key_log: print(f"L 点击: q_deg[1]={q_deg[1]} (J2)")

        # J3 (肘部俯仰, control_joints[2]=joint3)
        if _down_ascii(keys, 'u'):
            rate[2] += rate_unit
            if key_log: print(f"U 按住: rate[2]={rate[2]} (J3)")
        if _down_ascii(keys, 'o'):
            rate[2] -= rate_unit
            if key_log: print(f"O 按住: rate[2]={rate[2]} (J3)")
        if _hit_ascii(keys, 'u'):
            q_deg[2] += tap_step
            if key_log: print(f"U 点击: q_deg[2]={q_deg[2]} (J3)")
        if _hit_ascii(keys, 'o'):
            q_deg[2] -= tap_step
            if key_log: print(f"O 点击: q_deg[2]={q_deg[2]} (J3)")

        # 角速度积分 + 限位（j1 无限位，j2/j3 有限位）
        for i in range(3):
            q_deg[i] += rate[i] * dt
            # j1（索引0）可以360度无限旋转，不做限位
            if i != 0:
                q_deg[i] = max(limits_lo[i], min(limits_hi[i], q_deg[i]))
        
        # 轨道角度更新
        track_angle += track_rate * dt
        # 轨道角度归一化到 [-π, π]
        track_angle = (track_angle + math.pi) % (2 * math.pi) - math.pi
        
        # 🔍 调试日志（放在所有更新之后）
        if key_log and keys:
            ev = {k:int(v) for k,v in keys.items() if v & (p.KEY_WAS_TRIGGERED|p.KEY_IS_DOWN)}
            if ev:
                print("[Keys]", ev)
                print(f"  rate={[round(r,1) for r in rate]}, q_deg={[round(q,1) for q in q_deg]}")
                print(f"  lift_rate={round(lift_rate,2)}, lift_height={round(lift_height,3)}")
        
        # 更新机械臂基座位置（沿轨道移动）- 使用约束更新
        arm_x = track_radius * math.cos(track_angle)
        arm_y = track_radius * math.sin(track_angle)
        p.changeConstraint(
            constraint_id,
            jointChildPivot=[arm_x, arm_y, arm_z],
            jointChildFrameOrientation=p.getQuaternionFromEuler([0, 0, track_angle]),
            maxForce=10000
        )
        
        # 🆕 已移除调试线更新代码，实体支撑杆会自动显示

        # ====== 升降关节控制（使用位置控制+速度限制）======
        # 使用 POSITION_CONTROL 模式，让电机追踪 lift_height 目标位置
        p.setJointMotorControl2(
            arm_id, 
            lift_joint_idx, 
            p.POSITION_CONTROL,
            targetPosition=lift_height,
            positionGain=0.8,      # 位置增益（降低到0.8，避免抖动）
            velocityGain=1.0,      # 速度阻尼（适中阻尼，平衡响应和稳定）
            force=5000,            # 最大推力（足够推动机械臂）
            maxVelocity=1.5        # 最大速度限制（稍微降低，更平滑）
        )
        
        # 控制旋转关节 - 优化PD参数消除惯性感
        q_rad = [math.radians(v) for v in q_deg]
        p.setJointMotorControlArray(
            arm_id, control_joints, p.POSITION_CONTROL,
            targetPositions=q_rad,
            positionGains=[0.3, 0.3, 0.3],    # 降低P增益，避免过冲
            velocityGains=[1.0, 1.0, 1.0],    # 提高D增益，快速消除惯性
            forces=[180, 180, 180]
        )
        
        # 🆕 末端摄像机渲染（按C键开启后生效）
        if camera_enabled:
            camera_frame_counter += 1
            # 每N帧渲染一次，减少性能开销
            if camera_frame_counter >= camera_frame_skip:
                camera_frame_counter = 0
                
                # 获取末端执行器（ee）的世界坐标和方向
                ee_link_idx = -1  # 末端link索引
                for i in range(p.getNumJoints(arm_id)):
                    link_info = p.getJointInfo(arm_id, i)
                    if link_info[12].decode('utf-8') == 'ee':
                        ee_link_idx = i
                        break
                
                if ee_link_idx != -1:
                    # 获取末端link的状态
                    link_state = p.getLinkState(arm_id, ee_link_idx, computeForwardKinematics=True)
                    ee_pos = link_state[4]  # 世界坐标位置
                    ee_orn = link_state[5]  # 世界坐标姿态（四元数）
                    
                    # 将四元数转换为旋转矩阵
                    rotation_matrix = p.getMatrixFromQuaternion(ee_orn)
                    
                    # 摄像机朝向：假设末端坐标系的 +X 方向为摄像机朝向
                    camera_forward = [rotation_matrix[0], rotation_matrix[3], rotation_matrix[6]]
                    camera_up = [rotation_matrix[2], rotation_matrix[5], rotation_matrix[8]]  # +Z 为上方
                    
                    # 计算摄像机目标点（沿朝向延伸）
                    target_distance = 0.5  # 观察距离0.5米
                    camera_target = [
                        ee_pos[0] + camera_forward[0] * target_distance,
                        ee_pos[1] + camera_forward[1] * target_distance,
                        ee_pos[2] + camera_forward[2] * target_distance
                    ]
                    
                    # 优化：降低分辨率提升性能
                    width, height = 160, 120  # 降低到160x120（原来320x240）
                    fov = 60  # 视场角
                    aspect = width / height
                    near_plane = 0.01
                    far_plane = 2.0
                    
                    # 构建视图矩阵和投影矩阵
                    view_matrix = p.computeViewMatrix(
                        cameraEyePosition=ee_pos,
                        cameraTargetPosition=camera_target,
                        cameraUpVector=camera_up
                    )
                    proj_matrix = p.computeProjectionMatrixFOV(
                        fov=fov,
                        aspect=aspect,
                        nearVal=near_plane,
                        farVal=far_plane
                    )
                    
                    # 🚀 使用GPU硬件加速渲染（Metal on macOS）
                    # ER_BULLET_HARDWARE_OPENGL 会自动使用系统GPU（M4芯片的Metal）
                    img_arr = p.getCameraImage(
                        width, height,
                        viewMatrix=view_matrix,
                        projectionMatrix=proj_matrix,
                        renderer=p.ER_BULLET_HARDWARE_OPENGL,  # 使用硬件OpenGL（Metal后端）
                        flags=p.ER_NO_SEGMENTATION_MASK  # 🎯 关闭分割掩码提升性能
                    )
                    
                    # 🆕 保存摄像头图像到文件（供外部控制使用）
                    try:
                        from PIL import Image
                        # img_arr[2] 是 RGB 数据
                        rgb_array = np.array(img_arr[2], dtype=np.uint8)
                        rgb_array = np.reshape(rgb_array, (height, width, 4))[:, :, :3]  # 去除 alpha 通道
                        
                        # 保存到固定路径
                        camera_save_dir = "/tmp/pybullet_camera"
                        os.makedirs(camera_save_dir, exist_ok=True)
                        camera_image_path = os.path.join(camera_save_dir, "latest.png")
                        
                        img = Image.fromarray(rgb_array)
                        img.save(camera_image_path)
                    except Exception as e:
                        pass  # 静默失败，不影响仿真
                    
                    # 获取深度数据进行简单的目标检测
                    # img_arr[3] 是深度缓冲
                    # 可以通过深度判断是否有物体在视野中
                    import numpy as np
                    depth_buffer = np.array(img_arr[3])
                    
                    # 统计近距离像素（< 0.8m的物体）
                    near_pixels = np.sum(depth_buffer < 0.8)
                    if near_pixels > 100:  # 如果有足够多的近距离像素
                        # 计算平均深度
                        near_mask = depth_buffer < 0.8
                        avg_depth = np.mean(depth_buffer[near_mask])
                        # 线性化深度
                        real_distance = far_plane * near_plane / (far_plane - (far_plane - near_plane) * avg_depth)
                        # 降低打印频率，避免刷屏
                        if camera_frame_counter == 0:  # 只在渲染帧打印
                            print(f"📷 [Camera] 检测到物体！距离: {real_distance:.3f}m | 像素: {near_pixels}")
        
        # 🆕 更新机械臂状态文件（每 60 帧更新一次，约 0.25 秒）
        if p.isConnected() and hasattr(p, '_state_update_counter'):
            p._state_update_counter = getattr(p, '_state_update_counter', 0) + 1
        else:
            if not hasattr(p, '_state_update_counter'):
                p._state_update_counter = 0
            p._state_update_counter += 1
        
        if p._state_update_counter >= 60:
            p._state_update_counter = 0
            try:
                import json
                
                # 获取末端执行器位置
                ee_link_idx = -1
                for i in range(p.getNumJoints(arm_id)):
                    link_info = p.getJointInfo(arm_id, i)
                    if link_info[12].decode('utf-8') == 'ee':
                        ee_link_idx = i
                        break
                
                ee_pos = [0, 0, 0]
                if ee_link_idx != -1:
                    link_state = p.getLinkState(arm_id, ee_link_idx, computeForwardKinematics=True)
                    ee_pos = link_state[4]
                
                # 计算与果实的距离
                fruit_pos = [0.0, 0.0, 0.76]
                distance_to_fruit = math.sqrt(
                    (ee_pos[0] - fruit_pos[0])**2 +
                    (ee_pos[1] - fruit_pos[1])**2 +
                    (ee_pos[2] - fruit_pos[2])**2
                )
                
                # 构建状态字典
                state_data = {
                    "timestamp": time.time(),
                    "track_angle": math.degrees(track_angle),
                    "base_x": arm_x,
                    "base_y": arm_y,
                    "base_z": arm_z,
                    "lift_height": lift_height,
                    "j1_angle": q_deg[0],
                    "j2_angle": q_deg[1],
                    "j3_angle": q_deg[2],
                    "ee_x": ee_pos[0],
                    "ee_y": ee_pos[1],
                    "ee_z": ee_pos[2],
                    "distance_to_fruit": distance_to_fruit,
                    "fruit_position": fruit_pos
                }
                
                # 写入状态文件
                with open(state_file, "w", encoding="utf-8") as f:
                    json.dump(state_data, f, indent=2)
            except Exception as e:
                pass  # 静默失败

        p.stepSimulation()        # 👈 这里有！
        time.sleep(1/240.0)  # 保持 240Hz 仿真频率


if __name__ == "__main__":
    main()
