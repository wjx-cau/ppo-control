#!/usr/bin/env python3
"""
按键测试工具 - 检查键盘映射是否正确
"""
import pybullet as p
import pybullet_data
import time

# 连接到PyBullet GUI
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(p.pybullet_data.getDataPath())

# 创建一个简单的方块用于测试
box_id = p.loadURDF("cube.urdf", [0, 0, 1])

print("=" * 60)
print("按键测试工具")
print("=" * 60)
print("请按以下按键进行测试：")
print("  ⬆️ / ⬇️   - 上下箭头键")
print("  I / K    - J1旋转")
print("  J / L    - J2摆动")
print("  U / O    - J3摆动")
print("  Q / P    - 轨道旋转")
print("  C        - 摄像机开关")
print("  H        - 日志开关")
print("按 ESC 退出")
print("=" * 60)

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

while p.isConnected():
    keys = p.getKeyboardEvents()
    
    if keys:
        # 上下箭头
        if _down_code(keys, p.B3G_UP_ARROW):
            print("⬆️  按住 - 上箭头")
        if _hit_code(keys, p.B3G_UP_ARROW):
            print("⬆️  点击 - 上箭头")
        if _down_code(keys, p.B3G_DOWN_ARROW):
            print("⬇️  按住 - 下箭头")
        if _hit_code(keys, p.B3G_DOWN_ARROW):
            print("⬇️  点击 - 下箭头")
        
        # I/K键
        if _down_ascii(keys, 'i'):
            print("I   按住 - J1正向")
        if _hit_ascii(keys, 'i'):
            print("I   点击 - J1正向")
        if _down_ascii(keys, 'k'):
            print("K   按住 - J1反向")
        if _hit_ascii(keys, 'k'):
            print("K   点击 - J1反向")
        
        # J/L键
        if _down_ascii(keys, 'j'):
            print("J   按住 - J2正向")
        if _hit_ascii(keys, 'j'):
            print("J   点击 - J2正向")
        if _down_ascii(keys, 'l'):
            print("L   按住 - J2反向")
        if _hit_ascii(keys, 'l'):
            print("L   点击 - J2反向")
        
        # U/O键
        if _down_ascii(keys, 'u'):
            print("U   按住 - J3正向")
        if _hit_ascii(keys, 'u'):
            print("U   点击 - J3正向")
        if _down_ascii(keys, 'o'):
            print("O   按住 - J3反向")
        if _hit_ascii(keys, 'o'):
            print("O   点击 - J3反向")
        
        # Q/P键
        if _down_ascii(keys, 'q'):
            print("Q   按住 - 轨道逆时针")
        if _hit_ascii(keys, 'q'):
            print("Q   点击 - 轨道逆时针")
        if _down_ascii(keys, 'p'):
            print("P   按住 - 轨道顺时针")
        if _hit_ascii(keys, 'p'):
            print("P   点击 - 轨道顺时针")
        
        # C/H键
        if _hit_ascii(keys, 'c'):
            print("C   切换 - 摄像机")
        if _hit_ascii(keys, 'h'):
            print("H   切换 - 日志")
        
        # ESC退出
        if keys.get(p.B3G_ESCAPE, 0) & p.KEY_WAS_TRIGGERED:
            print("ESC - 退出")
            break
    
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()
print("测试完成！")
