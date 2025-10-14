#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyBullet 3-DoF 机械臂（GUI + 滑块/键盘并存，低延迟手感 + 稳健按键）

用法：
  conda activate bullet312
  python test.py

操作：
  - 滑块：窗口右侧 User Parameters 面板 J1/J2/J3。
  - 键盘（窗口需有焦点）：
      J1：I/K 或 ←/→
      J2：J/L 或 ↑/↓
      J3：U/O 或 PageUp/PageDown
    支持“按住连续变化”和“点按步进”(5°/次)。

说明：
  - 键盘采用“角速度积分 + 点按步进”的双机制，解决 macOS 下某些字母键只触发一次导致的“小抖一下”。
  - 滑块与键盘并存：只有当滑块真的移动（超过 0.2°）时，才覆盖目标角度。
"""

import os
import time
import math
import tempfile
import pybullet as p
import pybullet_data

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
    p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=40, cameraPitch=-35,
                                 cameraTargetPosition=[0.2, 0.0, 0.15])
    try:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    except Exception:
        pass

    p.loadURDF("plane.urdf")

    urdf_path = write_urdf_tmp(URDF_TEMPLATE)
    arm_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True,
                        flags=p.URDF_USE_SELF_COLLISION)
    assert isinstance(arm_id, int), "arm_id not set"
    print(f"arm_id={arm_id}")

    num_joints = p.getNumJoints(arm_id)
    print(f"num_joints={num_joints}")
    control_joints = [0, 1, 2]

    # 坐标轴辅助
    p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [1, 0, 0], lineWidth=3, parentObjectUniqueId=arm_id, parentLinkIndex=-1)
    p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0], lineWidth=3, parentObjectUniqueId=arm_id, parentLinkIndex=-1)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [0, 0, 1], lineWidth=3, parentObjectUniqueId=arm_id, parentLinkIndex=-1)

    # 叶片示意
    leaf_thickness = 0.004
    leaf_half_extents = [0.12, 0.08, leaf_thickness / 2]
    leaf_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=leaf_half_extents, rgbaColor=[0.2, 0.7, 0.2, 1])
    leaf_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=leaf_half_extents)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=leaf_col, baseVisualShapeIndex=leaf_vis,
                      basePosition=[0.35, 0.0, 0.35], baseOrientation=p.getQuaternionFromEuler([0, 0.0, 0]))

    # 降低阻尼，提响应
    for link in range(-1, num_joints):
        try:
            p.changeDynamics(arm_id, link, linearDamping=0.0, angularDamping=0.0)
        except Exception:
            pass

    # 演示 3 秒
    t0 = time.time()
    while time.time() - t0 < 3.0:
        t = time.time() - t0
        q1 = math.radians(20.0 * math.sin(1.5 * t))
        q2 = math.radians(30.0 + 15.0 * math.sin(1.7 * t))
        q3 = math.radians(40.0 * math.sin(2.0 * t))
        p.setJointMotorControlArray(arm_id, control_joints, p.POSITION_CONTROL,
                                    targetPositions=[q1, q2, q3], positionGains=[0.45]*3, forces=[40]*3)
        p.stepSimulation()
        time.sleep(1/240)

    # 创建滑块
    slider_ids = []
    try:
        slider_ids = [
            p.addUserDebugParameter("J1 yaw (deg)", -180, 180, 0),
            p.addUserDebugParameter("J2 pitch (deg)", -100, 100, 30),
            p.addUserDebugParameter("J3 pitch (deg)", -120, 120, 45),
        ]
        time.sleep(0.2)
        print("Slider IDs:", slider_ids)
    except Exception as e:
        print("[Warn] Failed to create sliders:", e)

    # 实时仿真，提升交互
    try:
        p.setRealTimeSimulation(1)
    except Exception:
        pass

    print("Switch to interactive control.")
    print("[Hint] Keyboard & sliders: J1(I/K or A/D or ←/→), J2(J/L or W/S or ↑/↓), J3(U/O or Q/E or PgUp/PgDn)")
    print("[Hint] Debug: press 'H' to toggle key logging, 'P' to print joint states")

    # 目标角度（度），优先用滑块初值
    def read_slider_vals():
        vals = []
        for sid in slider_ids:
            try:
                vals.append(p.readUserDebugParameter(sid))
            except Exception:
                return None
        return vals if len(vals) == 3 and all(v is not None for v in vals) else None

    q_deg = read_slider_vals() or [0.0, 30.0, 45.0]
    last_slider = q_deg[:]

    # 键盘角速度（deg/s）与点按步进
    rate_unit = 120.0           # 按住时的角速度
    tap_step = 5.0              # 点按一次的步进
    limits_lo = [-180.0, -100.0, -120.0]
    limits_hi = [ 180.0,  100.0,  120.0]

    # 调试开关
    key_log = False

    # 工具：检测按住/点按（ASCII 与特殊键）
    def _down_ascii(keys, ch):
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

        # 若滑块被移动（超过阈值），覆盖目标
        vals = read_slider_vals()
        if vals is not None and any(abs(v - ls) > 0.2 for v, ls in zip(vals, last_slider)):
            q_deg = vals[:]
            last_slider = vals[:]

        keys = p.getKeyboardEvents()

        # 调试：按 H 开关日志，按 P 打印关节状态
        if _hit_ascii(keys, 'h'):
            key_log = not key_log
            print("[KeyLog]", "ON" if key_log else "OFF")
        if _hit_ascii(keys, 'p'):
            js = [p.getJointState(arm_id, i)[0] for i in control_joints]
            print("[Joints rad]", [round(x,3) for x in js])

        if key_log and keys:
            # 仅打印触发事件，避免刷屏
            ev = {k:int(v) for k,v in keys.items() if v & (p.KEY_WAS_TRIGGERED|p.KEY_IS_DOWN)}
            if ev:
                print("[Keys]", ev)

        rate = [0.0, 0.0, 0.0]

        # J1：I/K 或 A/D 或 ←/→
        if _down_ascii(keys, 'i') or _down_ascii(keys, 'd') or _down_code(keys, p.B3G_RIGHT_ARROW):
            rate[0] += rate_unit
        if _down_ascii(keys, 'k') or _down_ascii(keys, 'a') or _down_code(keys, p.B3G_LEFT_ARROW):
            rate[0] -= rate_unit
        if _hit_ascii(keys, 'i') or _hit_ascii(keys, 'd') or _hit_code(keys, p.B3G_RIGHT_ARROW):
            q_deg[0] += tap_step
        if _hit_ascii(keys, 'k') or _hit_ascii(keys, 'a') or _hit_code(keys, p.B3G_LEFT_ARROW):
            q_deg[0] -= tap_step

        # J2：J/L 或 W/S 或 ↑/↓
        if _down_ascii(keys, 'j') or _down_ascii(keys, 'w') or _down_code(keys, p.B3G_UP_ARROW):
            rate[1] += rate_unit
        if _down_ascii(keys, 'l') or _down_ascii(keys, 's') or _down_code(keys, p.B3G_DOWN_ARROW):
            rate[1] -= rate_unit
        if _hit_ascii(keys, 'j') or _hit_ascii(keys, 'w') or _hit_code(keys, p.B3G_UP_ARROW):
            q_deg[1] += tap_step
        if _hit_ascii(keys, 'l') or _hit_ascii(keys, 's') or _hit_code(keys, p.B3G_DOWN_ARROW):
            q_deg[1] -= tap_step

        # J3：U/O 或 Q/E 或 PgUp/PgDn
        if _down_ascii(keys, 'u') or _down_ascii(keys, 'q') or _down_code(keys, p.B3G_PAGE_UP):
            rate[2] += rate_unit
        if _down_ascii(keys, 'o') or _down_ascii(keys, 'e') or _down_code(keys, p.B3G_PAGE_DOWN):
            rate[2] -= rate_unit
        if _hit_ascii(keys, 'u') or _hit_ascii(keys, 'q') or _hit_code(keys, p.B3G_PAGE_UP):
            q_deg[2] += tap_step
        if _hit_ascii(keys, 'o') or _hit_ascii(keys, 'e') or _hit_code(keys, p.B3G_PAGE_DOWN):
            q_deg[2] -= tap_step

        # 角速度积分 + 限位
        for i in range(3):
            q_deg[i] += rate[i] * dt
            q_deg[i] = max(limits_lo[i], min(limits_hi[i], q_deg[i]))

        q_rad = [math.radians(v) for v in q_deg]
        p.setJointMotorControlArray(
            arm_id, control_joints, p.POSITION_CONTROL,
            targetPositions=q_rad,
            positionGains=[1.2, 1.2, 1.2],
            velocityGains=[0.6, 0.6, 0.6],
            forces=[180, 180, 180]
        )

        time.sleep(0.005)


if __name__ == "__main__":
    main()
