#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用 Qwen 模型控制 PyBullet 机械臂
通过自然语言指令让 Qwen 生成键盘控制序列
"""

import os
import sys
import json
import subprocess
import time
import threading
import tempfile

# ======== Qwen 模型配置 ========
MODEL_NAME = "lmstudio-community/Qwen3-VL-8B-Instruct-MLX-8bit"

# ======== 机械臂控制说明（给 Qwen 的系统提示词）========
SYSTEM_PROMPT = """你是一个机械臂控制专家助手。你需要根据用户的指令，生成键盘控制序列来操作 PyBullet 仿真环境中的机械臂。

# 机械臂硬件配置：
- 机械臂安装在圆形轨道上（半径 0.5 米）
- 轨道中心有一株植物（高度约 0.65 米）
- 机械臂有 4 个自由度：
  1. 升降关节（Lift）：0-0.5米垂直移动
  2. J1 底座关节：360度无限旋转（安装在升降平台上）
  3. J2 肩部关节：±100度俯仰
  4. J3 肘部关节：±120度俯仰

# 键盘控制方式：
## 轨道移动（让机械臂绕植物转圈）：
- 'Q' 键：逆时针旋转（沿轨道向左移动）
- 'P' 键：顺时针旋转（沿轨道向右移动）

## 升降控制：
- 'Y' 键：向上升降
- 'H' 键：向下升降

## 机械臂关节控制：
- 'I' 键：J1 底座逆时针旋转
- 'K' 键：J1 底座顺时针旋转
- 'J' 键：J2 肩部向上
- 'L' 键：J2 肩部向下
- 'U' 键：J3 肘部向上
- 'O' 键：J3 肘部向下

## 调试与摄像机：
- 'G' 键：开关按键日志
- 'T' 键：打印当前状态
- 'C' 键：开启/关闭末端摄像机
- 'V' 键：调节摄像机渲染频率

# 控制说明：
- 按住按键：连续变化（角速度 120°/秒，升降速度 0.5m/s）
- 点按按键：步进变化（5°/次，升降 5cm/次）
- 轨道旋转速度：约 28.6°/秒（按住时）或 10°/次（点按）

# 你的任务：
根据用户的自然语言指令，生成一个 JSON 格式的控制序列，包含：
1. 每个动作的键盘按键
2. 按键持续时间（秒）
3. 动作描述

# 输出格式（严格的 JSON 数组）：
```json
[
  {"key": "Q", "duration": 2.0, "description": "逆时针旋转 2 秒（约 57 度）"},
  {"key": "Y", "duration": 1.0, "description": "升高 1 秒（约 0.5 米）"},
  {"key": "WAIT", "duration": 0.5, "description": "等待 0.5 秒"}
]
```

# 注意事项：
- 只输出 JSON 数组，不要有其他文字
- duration 是浮点数，单位为秒
- 如果需要停顿，使用 {"key": "WAIT", "duration": X}
- 绕植物一周需要旋转 360 度，按住 Q 键约需 12.6 秒
- 计算时考虑动作的连续性和平滑性
"""

# ======== 调用 Qwen 生成控制序列 ========
def call_qwen_for_control(user_instruction: str) -> list:
    """
    调用 Qwen 模型，根据用户指令生成控制序列
    返回: list of dict, 每个 dict 包含 {key, duration, description}
    """
    full_prompt = f"{SYSTEM_PROMPT}\n\n用户指令：{user_instruction}\n\n请生成控制序列（只输出 JSON 数组）："
    
    # 保存提示词到临时文件（避免命令行长度限制）
    with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False, encoding='utf-8') as f:
        prompt_file = f.name
        f.write(full_prompt)
    
    try:
        # 调用 mlx_lm.generate（文本生成，不需要图片）
        cmd = [
            sys.executable, "-m", "mlx_lm.generate",
            "--model", MODEL_NAME,
            "--prompt", full_prompt,
            "--max-tokens", "1024",
            "--temp", "0.3"  # 降低温度，让输出更稳定
        ]
        
        print(f"🤖 正在调用 Qwen 模型生成控制序列...")
        print(f"   模型: {MODEL_NAME}")
        print(f"   指令: {user_instruction}\n")
        
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=60
        )
        
        output = result.stdout.strip()
        
        if result.returncode != 0:
            print(f"❌ Qwen 调用失败：{result.stderr}")
            return []
        
        print(f"📝 Qwen 原始输出：\n{output}\n")
        
        # 解析 JSON（去除可能的代码块标记）
        import re
        cleaned = re.sub(r"```(?:json)?", "", output, flags=re.IGNORECASE).strip()
        
        # 尝试提取 JSON 数组
        json_candidates = re.findall(r'\[.*?\]', cleaned, flags=re.DOTALL)
        
        for candidate in json_candidates:
            try:
                data = json.loads(candidate)
                if isinstance(data, list) and len(data) > 0:
                    print(f"✅ 成功解析控制序列，共 {len(data)} 个动作\n")
                    return data
            except json.JSONDecodeError:
                continue
        
        print(f"⚠️ 无法解析 JSON，尝试手动提取...")
        # 如果无法解析，返回空列表
        return []
    
    except subprocess.TimeoutExpired:
        print("❌ Qwen 调用超时（60秒）")
        return []
    except FileNotFoundError:
        print("❌ 找不到 mlx_lm.generate，请先安装：pip install mlx-lm")
        return []
    except Exception as e:
        print(f"❌ 调用 Qwen 时发生错误：{e}")
        return []
    finally:
        # 清理临时文件
        try:
            os.unlink(prompt_file)
        except Exception:
            pass


# ======== 通过文件通信控制模拟器 ========
class SimulatorController:
    """
    控制器：通过文件通信向 PyBullet 模拟器发送按键命令
    """
    def __init__(self):
        self.command_file = "/tmp/pybullet_arm_command.txt"
        self.status_file = "/tmp/pybullet_arm_status.txt"
        self.simulator_process = None
        
    def wait_for_simulator(self, timeout=30):
        """等待模拟器就绪"""
        print("⏳ 等待模拟器启动...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if os.path.exists(self.status_file):
                try:
                    with open(self.status_file, "r") as f:
                        status = f.read().strip()
                        if status == "READY":
                            print("✅ 模拟器已就绪\n")
                            return True
                except Exception:
                    pass
            time.sleep(0.5)
        
        print("❌ 模拟器启动超时")
        return False
    
    def send_command(self, key: str, duration: float):
        """
        发送按键命令到模拟器
        key: 按键字符（如 'Q', 'P', 'Y' 等）
        duration: 持续时间（秒）
        """
        try:
            with open(self.command_file, "w", encoding="utf-8") as f:
                f.write(f"{key.upper()} {duration}\n")
            return True
        except Exception as e:
            print(f"❌ 发送命令失败：{e}")
            return False
    
    def execute_sequence(self, control_sequence: list):
        """执行控制序列"""
        print(f"\n{'='*50}")
        print(f"🚀 开始执行控制序列（共 {len(control_sequence)} 个动作）")
        print(f"{'='*50}\n")
        
        for i, action in enumerate(control_sequence, 1):
            key = action.get("key", "WAIT")
            duration = action.get("duration", 0.0)
            desc = action.get("description", "")
            
            print(f"[{i}/{len(control_sequence)}] {desc}")
            
            if key == "WAIT":
                print(f"   ⏸️  等待 {duration:.2f} 秒...")
                time.sleep(duration)
            else:
                print(f"   🎮 发送命令: {key} 持续 {duration:.2f}s")
                if self.send_command(key, duration):
                    print(f"   ✓ 命令已发送")
                    # 等待命令执行完成
                    time.sleep(duration + 0.1)  # 额外等待 0.1 秒确保命令执行完成
                else:
                    print(f"   ✗ 命令发送失败")
            
            print()
        
        print(f"{'='*50}")
        print(f"✅ 控制序列执行完成")
        print(f"{'='*50}\n")
    
    def cleanup(self):
        """清理临时文件"""
        try:
            if os.path.exists(self.command_file):
                os.remove(self.command_file)
            if os.path.exists(self.status_file):
                os.remove(self.status_file)
        except Exception:
            pass


# ======== 备用方案：直接生成硬编码序列 ========
def generate_fallback_circle_sequence() -> list:
    """
    备用方案：如果 Qwen 无法生成有效序列，使用硬编码的绕圈序列
    """
    print("⚠️ 使用备用方案：硬编码控制序列\n")
    
    # 绕植物一周（360度）：按住 Q 键约 12.6 秒
    # 为了更平滑，分成多段
    return [
        {"key": "WAIT", "duration": 0.5, "description": "开始前等待 0.5 秒"},
        {"key": "Q", "duration": 3.0, "description": "逆时针旋转 3 秒（约 85.8 度）"},
        {"key": "WAIT", "duration": 0.2, "description": "短暂停顿"},
        {"key": "Q", "duration": 3.0, "description": "继续逆时针旋转 3 秒（约 85.8 度）"},
        {"key": "WAIT", "duration": 0.2, "description": "短暂停顿"},
        {"key": "Q", "duration": 3.0, "description": "继续逆时针旋转 3 秒（约 85.8 度）"},
        {"key": "WAIT", "duration": 0.2, "description": "短暂停顿"},
        {"key": "Q", "duration": 3.6, "description": "最后逆时针旋转 3.6 秒（约 103.2 度）"},
        {"key": "WAIT", "duration": 1.0, "description": "完成，等待 1 秒"}
    ]


# ======== 主函数 ========
def main():
    print(f"\n{'='*60}")
    print(f"🤖 Qwen 机械臂控制系统")
    print(f"{'='*60}\n")
    
    # 用户指令
    user_instruction = "让机械臂沿轨道逆时针绕植物移动一周（360度），要平滑连续"
    
    print(f"📋 用户指令：{user_instruction}\n")
    
    # 1. 检查模拟器是否已启动
    controller = SimulatorController()
    
    status_file_exists = os.path.exists(controller.status_file)
    
    if not status_file_exists:
        print("⚠️ 模拟器未启动！")
        print("📌 请先在另一个终端运行：")
        print("   python 'pybullt/ test_environment.py'\n")
        print("💡 等待演示动画结束（约3秒）后，模拟器会创建状态文件。")
        print("   然后再次运行本脚本。\n")
        return
    
    # 2. 等待模拟器就绪
    if not controller.wait_for_simulator():
        print("❌ 无法连接到模拟器")
        return
    
    # 3. 调用 Qwen 生成控制序列
    print("🤖 正在调用 Qwen 生成控制序列...\n")
    control_sequence = call_qwen_for_control(user_instruction)
    
    # 4. 如果 Qwen 返回空，使用备用方案
    if not control_sequence:
        control_sequence = generate_fallback_circle_sequence()
    
    # 5. 显示生成的序列
    print(f"📊 生成的控制序列：")
    print(json.dumps(control_sequence, ensure_ascii=False, indent=2))
    print()
    
    # 6. 显示执行时间线
    print(f"{'='*60}")
    print(f"📋 执行时间线预览：")
    print(f"{'='*60}\n")
    
    total_time = 0.0
    for i, action in enumerate(control_sequence, 1):
        key = action.get("key", "WAIT")
        duration = action.get("duration", 0.0)
        desc = action.get("description", "")
        
        print(f"[T+{total_time:5.1f}s] {desc}")
        total_time += duration
    
    print(f"\n总用时：{total_time:.1f} 秒")
    print(f"{'='*60}\n")
    
    # 7. 确认执行
    try:
        user_input = input("� 按 Enter 开始执行，或输入 'n' 取消: ").strip().lower()
        if user_input == 'n':
            print("❌ 已取消执行")
            return
    except KeyboardInterrupt:
        print("\n❌ 已取消执行")
        return
    
    # 8. 执行控制序列
    try:
        controller.execute_sequence(control_sequence)
        print("🎉 任务完成！")
    except KeyboardInterrupt:
        print("\n⚠️ 执行被中断")
    finally:
        # 不清理临时文件，让模拟器继续运行
        pass


if __name__ == "__main__":
    main()
