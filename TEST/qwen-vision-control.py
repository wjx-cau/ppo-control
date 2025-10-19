#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qwen 视觉控制系统 - 闭环交互版本
通过摄像头视觉反馈，让 Qwen 控制机械臂寻找并对准植物果实
"""

import os
import sys
import json
import subprocess
import time
import tempfile
import re
from PIL import Image
import numpy as np

# 强制离线模式，避免访问 Hugging Face
os.environ['HF_HUB_OFFLINE'] = '1'
os.environ['TRANSFORMERS_OFFLINE'] = '1'

# ======== 配置 ========
# 模型路径：使用本地路径避免网络问题
# 在线模式（需要网络）：
# MODEL_NAME = "lmstudio-community/Qwen3-VL-8B-Instruct-MLX-8bit"
# 离线模式（推荐，无需网络）：
MODEL_NAME = "/Users/wjx_macair/.cache/huggingface/hub/models--lmstudio-community--Qwen3-VL-8B-Instruct-MLX-8bit/snapshots/ef2f0ae5bd3b1f48193bf2622dd1f654af691e75"
COMMAND_FILE = "/tmp/pybullet_arm_command.txt"
STATUS_FILE = "/tmp/pybullet_arm_status.txt"
CAMERA_IMAGE_DIR = "/tmp/pybullet_camera"  # 摄像头图像保存目录
CAMERA_IMAGE_PATH = "/tmp/pybullet_camera/latest.png"  # 最新图像

# 输出目录配置
OUTPUT_BASE_DIR = "/Users/wjx_macair/Desktop/code/TEST/QWEN-OUTPUT"

# 创建图像目录
os.makedirs(CAMERA_IMAGE_DIR, exist_ok=True)
os.makedirs(OUTPUT_BASE_DIR, exist_ok=True)

# ======== Qwen 视觉系统提示词 ========
VISION_SYSTEM_PROMPT = """你是机械臂末端的摄像头AI。

# 任务目标
找到并对准绿色植物，让植物在画面中心。

# 环境说明
- 紫色墙壁 = 天空
- 蓝色棋盘 = 地面
- 黑色圆盘 = 植物底座
- 绿色物体 = 植物（目标）

# 控制按键（速度：120°/秒）
- I键：底座左转 | K键：底座右转
- J键：抬头看 | L键：低头看
- U键：微调向上 | O键：微调向下
- Q键：轨道逆时针 | P键：轨道顺时针
- Y键：升高 | H键：降低

# 控制时长
- 大幅搜索：0.5~1.0秒（60°~120°）
- 小幅调整：0.15~0.3秒（18°~36°）

# 控制方式：如果植物在画面左侧，那么想让植物进入画面中心，你应该将摄像头向左移动，反之亦然。
# 输出格式
```json
{
  "summary": "描述你当前看到的内容",
  "control": {
    "key": "K",
    "duration": 0.3
  }
}
```

只输出JSON，不要其他文字。"""

# ======== 模拟器通信 ========
class SimulatorInterface:
    """与 PyBullet 模拟器通信"""
    
    def __init__(self):
        self.command_file = COMMAND_FILE
        self.status_file = STATUS_FILE
        self.state_file = "/tmp/pybullet_arm_state.json"
        self.camera_image_path = CAMERA_IMAGE_PATH
    
    def is_ready(self):
        """检查模拟器是否就绪"""
        if not os.path.exists(self.status_file):
            return False
        try:
            with open(self.status_file, "r") as f:
                return f.read().strip() == "READY"
        except Exception:
            return False
    
    def send_command(self, key: str, duration: float):
        """发送控制命令"""
        try:
            with open(self.command_file, "w", encoding="utf-8") as f:
                f.write(f"{key.upper()} {duration}\n")
            return True
        except Exception as e:
            print(f"❌ 发送命令失败：{e}")
            return False
    
    def enable_camera(self):
        """开启末端摄像头"""
        return self.send_command("C", 0.1)  # C 键开启摄像头
    
    def capture_image(self):
        """等待并获取最新的摄像头图像"""
        # 给模拟器时间渲染
        time.sleep(0.3)
        
        if not os.path.exists(self.camera_image_path):
            print(f"⚠️ 摄像头图像未找到：{self.camera_image_path}")
            return None
        
        try:
            # 读取图像
            img = Image.open(self.camera_image_path)
            # 添加时间戳保存历史
            timestamp = int(time.time())
            history_path = os.path.join(CAMERA_IMAGE_DIR, f"frame_{timestamp}.png")
            img.save(history_path)
            print(f"📸 已捕获图像：{history_path}")
            return img
        except Exception as e:
            print(f"❌ 读取图像失败：{e}")
            return None
    
    def get_robot_state(self):
        """读取机械臂当前状态"""
        try:
            if not os.path.exists(self.state_file):
                return None
            
            with open(self.state_file, "r", encoding="utf-8") as f:
                state = json.load(f)
            return state
        except Exception as e:
            print(f"⚠️ 读取状态失败：{e}")
            return None


# ======== Qwen 视觉分析 ========
class QwenVisionAnalyzer:
    """使用 Qwen-VL 分析图像（多轮对话）"""
    
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.conversation_history = []  # 存储对话历史
    
    def analyze_image(self, image_path: str, iteration: int, robot_state: dict = None) -> dict:
        """
        分析图像并返回控制建议（多轮对话模式）
        robot_state: 当前机械臂状态信息
        返回：解析后的 JSON 字典，如果失败返回 None
        """
        # 构建历史上下文（只告诉上次的控制动作，不告诉看到了什么）
        history_context = ""
        if self.conversation_history:
            last = self.conversation_history[-1]
            history_context = f"""

# 上次控制动作
我刚才执行了：{last['control_key']} 键 {last['control_duration']:.2f}秒

请观察当前画面，判断控制效果（例如：画面是否移动了？目标是否还在视野中？）# 控制方式：如果植物在画面左侧，那么想让植物进入画面中心，你应该将摄像头向左移动，反之亦然。"""
        else:
            history_context = """

# 第一次观察
这是你第一次看到画面，请描述看到的内容并开始探索。"""
        
        prompt = f"{VISION_SYSTEM_PROMPT}{history_context}\n\n请分析当前图像并输出控制指令："
        
        # 使用 conda 环境中的 Python（bullet312）
        conda_python = "/Users/wjx_macair/miniforge/envs/bullet312/bin/python"
        python_cmd = conda_python if os.path.exists(conda_python) else sys.executable
        
        cmd = [
            python_cmd, "-m", "mlx_vlm.generate",
            "--model", self.model_name,
            "--prompt", prompt,
            "--image", image_path,
            "--max-tokens", "600",
            "--temp", "0.3"
        ]
        
        print(f"🤖 正在调用 Qwen-VL 分析图像...")
        print(f"   使用 Python: {python_cmd}")
        
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=60
            )
            
            if result.returncode != 0:
                stderr = result.stderr
                print(f"❌ Qwen-VL 调用失败：")
                
                # 识别常见错误类型
                if "SSLError" in stderr or "huggingface.co" in stderr:
                    print("   ⚠️ 网络连接失败：无法访问 Hugging Face")
                    print("   💡 解决方法：")
                    print("      1. 确保 MODEL_NAME 使用本地路径")
                    print("      2. 检查网络连接")
                    print(f"      3. 当前模型路径：{self.model_name}")
                elif "FileNotFoundError" in stderr or "No such file" in stderr:
                    print("   ⚠️ 模型文件未找到")
                    print(f"   💡 请检查路径是否正确：{self.model_name}")
                else:
                    # 只显示最后 5 行错误信息
                    error_lines = stderr.strip().split('\n')
                    print(f"   详细错误：{chr(10).join(error_lines[-5:])}")
                
                return None
            
            output = result.stdout.strip()
            print(f"\n📝 Qwen-VL 原始输出：\n{output}\n")
            
            # 解析 JSON
            parsed = self._parse_json_response(output)
            
            if parsed:
                print(f"✅ 成功解析视觉分析结果")
                
                # 添加到对话历史（新格式：control + summary）
                control = parsed.get('control', {})
                self.conversation_history.append({
                    'iteration': iteration,
                    'summary': parsed.get('summary', ''),
                    'control_key': control.get('key', 'K'),
                    'control_duration': control.get('duration', 0.5)
                })
                
                return parsed
            else:
                print(f"⚠️ 无法解析 JSON 响应")
                return None
        
        except subprocess.TimeoutExpired:
            print("❌ Qwen-VL 调用超时")
            return None
        except FileNotFoundError:
            print("❌ 找不到 mlx_vlm，请安装：pip install mlx-vlm")
            return None
        except Exception as e:
            print(f"❌ Qwen-VL 分析失败：{e}")
            return None
    
    def _parse_json_response(self, output: str) -> dict:
        """解析 Qwen 输出中的 JSON"""
        # 提取 assistant 回复部分（去除模型输出的调试信息）
        assistant_match = re.search(r'<\|im_start\|>assistant\s*(.*?)(?:=====|$)', output, flags=re.DOTALL)
        if assistant_match:
            output = assistant_match.group(1)
        
        # 去除代码块标记
        cleaned = re.sub(r"```(?:json)?", "", output, flags=re.IGNORECASE).strip()
        
        # 尝试提取 JSON 对象
        json_matches = re.findall(r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}', cleaned, flags=re.DOTALL)
        
        # 从后往前尝试（最后一个JSON通常是实际输出，而不是提示词中的示例）
        for candidate in reversed(json_matches):
            try:
                data = json.loads(candidate)
                # 验证必需字段（新格式：summary + control）
                if "summary" in data and "control" in data:
                    # 排除提示词中的示例（检查是否是通用示例文本）
                    summary = data.get("summary", "")
                    if "一句话总结" not in summary and "示例" not in summary:
                        return data
                # 兼容旧格式
                elif "what_i_see" in data and "next_action" in data:
                    # 转换为新格式
                    return {
                        "summary": data.get("what_i_see", ""),
                        "control": data.get("next_action", {})
                    }
            except json.JSONDecodeError:
                continue
        
        return None


# ======== 视觉控制循环 ========
class VisionControlLoop:
    """视觉反馈控制循环"""
    
    def __init__(self, simulator: SimulatorInterface, analyzer: QwenVisionAnalyzer):
        self.simulator = simulator
        self.analyzer = analyzer
        self.max_iterations = 10  # 最大迭代次数
        self.history = []  # 记录历史动作
        self.output_dir = None  # 当前迭代输出目录
    
    def run(self):
        """运行视觉控制循环"""
        print(f"\n{'='*60}")
        print(f"🔭 Qwen 自主探索：观察环境并控制机械臂")
        print(f"{'='*60}\n")
        
        # 0. 创建本次运行的输出目录
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(OUTPUT_BASE_DIR, f"run_{timestamp}")
        os.makedirs(self.output_dir, exist_ok=True)
        print(f"📁 输出目录：{self.output_dir}\n")
        
        # 1. 开启摄像头
        print(f"📷 步骤 1：开启摄像头...")
        if not self.simulator.enable_camera():
            print("❌ 无法开启摄像头")
            return False
        print("✅ 摄像头已开启")
        print("🎬 Qwen 开始观察...\n")
        time.sleep(1.0)  # 等待摄像头稳定
        
        # 2. 迭代控制循环
        for iteration in range(1, self.max_iterations + 1):
            print(f"\n{'─'*60}")
            print(f"🔄 迭代 {iteration}/{self.max_iterations}")
            print(f"{'─'*60}\n")
            
            # 创建当前迭代的子目录
            iter_dir = os.path.join(self.output_dir, f"iteration_{iteration:02d}")
            os.makedirs(iter_dir, exist_ok=True)
            
            # 2.1 捕获图像
            print(f"📸 捕获摄像头图像...")
            image = self.simulator.capture_image()
            
            if image is None:
                print("⚠️ 无法获取图像，跳过本次迭代")
                time.sleep(1)
                continue
            
            # 保存图像到迭代目录
            image_path = os.path.join(iter_dir, "camera_view.png")
            image.save(image_path)
            print(f"   💾 已保存：{image_path}")
            
            # 2.2 读取机械臂状态
            robot_state = self.simulator.get_robot_state()
            
            if robot_state:
                camera_height = robot_state.get('base_z', 0.05) + robot_state.get('lift_height', 0) + 0.3
                print(f"   📍 高度: {camera_height:.2f}m, J1: {robot_state['j1_angle']:.0f}°")
                
                # 保存状态到文件
                state_path = os.path.join(iter_dir, "robot_state.json")
                with open(state_path, "w", encoding="utf-8") as f:
                    json.dump(robot_state, f, indent=2, ensure_ascii=False)
            
            # 2.3 Qwen 分析图像
            print(f"🧠 Qwen 分析中...")
            analysis = self.analyzer.analyze_image(image_path, iteration, robot_state)
            
            if analysis is None:
                print("⚠️ 分析失败，使用默认策略（旋转搜索）")
                analysis = self._fallback_search_strategy(iteration)
            
            # 保存分析结果到文件
            analysis_path = os.path.join(iter_dir, "analysis.json")
            with open(analysis_path, "w", encoding="utf-8") as f:
                json.dump(analysis, f, indent=2, ensure_ascii=False)
            print(f"   💾 分析结果已保存：{analysis_path}")
            
            # 2.3 显示分析结果
            self._print_analysis(analysis)
            
            # 2.4 记录历史
            self.history.append({
                "iteration": iteration,
                "analysis": analysis,
                "timestamp": time.time(),
                "output_dir": iter_dir
            })
            
            # 2.5 检查是否完成
            if analysis.get("exploration_done", False) or analysis.get("task_complete", False):
                print(f"\n✅ Qwen 认为探索已完成！")
                self._save_summary()
                return True
            
            # 2.6 执行控制动作
            control = analysis.get("control", analysis.get("next_action", {}))
            key = control.get("key", "K")
            duration = control.get("duration", 0.5)
            
            print(f"\n🎮 下一步动作：")
            print(f"   按键: {key}")
            print(f"   时长: {duration:.1f}s")
            print(f"   总结: {analysis.get('summary', '继续探索')}")
            
            # 保存控制命令
            command_path = os.path.join(iter_dir, "command.txt")
            with open(command_path, "w", encoding="utf-8") as f:
                f.write(f"Key: {key}\n")
                f.write(f"Duration: {duration}\n")
                f.write(f"Summary: {analysis.get('summary', '')}\n")
            
            if self.simulator.send_command(key, duration):
                print(f"   ✓ 命令已发送")
                # 等待动作完成 + 额外稳定时间
                time.sleep(duration + 0.5)
            else:
                print(f"   ✗ 命令发送失败")
        
        # 3. 达到最大迭代次数
        print(f"\n⚠️ 达到最大迭代次数（{self.max_iterations}），任务未完成")
        self._save_summary()
        return False
    
    def _fallback_search_strategy(self, iteration: int) -> dict:
        """备用搜索策略：缓慢旋转 J1 搜索植物"""
        return {
            "summary": f"分析失败，执行默认搜索策略（旋转 {iteration}）",
            "control": {
                "key": "I",  # 逆时针旋转 J1
                "duration": 1.0
            },
            "exploration_done": False
        }
    
    def _print_analysis(self, analysis: dict):
        """打印分析结果"""
        print(f"\n📊 Qwen 的观察和思考：")
        
        # 显示观察描述
        what_i_see = analysis.get('what_i_see', analysis.get('observation', ''))
        if what_i_see:
            print(f"   👁️  看到：{what_i_see}")
        
        # 显示分析
        analysis_text = analysis.get('analysis', '')
        if analysis_text:
            print(f"   🔍 分析：{analysis_text}")
        
        # 显示移动计划
        movement_plan = analysis.get('movement_plan', {})
        if movement_plan:
            print(f"   � 移动计划：")
            direction = movement_plan.get('direction', '')
            reason = movement_plan.get('reason', '')
            expected = movement_plan.get('expected_change', '')
            if direction:
                print(f"      方向：{direction}")
            if reason:
                print(f"      原因：{reason}")
            if expected:
                print(f"      预期：{expected}")
        
        # 显示发现的物体（兼容旧格式）
        objects = analysis.get('interesting_objects', [])
        if objects:
            print(f"   🔍 物体：{', '.join(objects)}")
        
        # 显示想法（兼容旧格式）
        thought = analysis.get('my_thought', '')
        if thought:
            print(f"   💭 想法：{thought}")
    def _print_analysis(self, analysis: dict):
        """打印分析结果"""
        print(f"\n📊 Qwen 的分析：")
        
        # 显示总结
        summary = analysis.get('summary', '')
        if summary:
            print(f"   💬 总结：{summary}")
    
    def _save_summary(self):
        """保存并打印探索总结"""
        print(f"\n{'='*60}")
        print(f"📈 探索总结")
        print(f"{'='*60}")
        print(f"总观察次数: {len(self.history)}")
        
        # 收集所有总结
        all_summaries = []
        all_controls = []
        
        for h in self.history:
            analysis = h['analysis']
            
            # 收集总结
            summary = analysis.get('summary', '')
            if summary:
                all_summaries.append(summary)
            
            # 收集控制
            control = analysis.get('control', {})
            all_controls.append(f"{control.get('key', '?')} {control.get('duration', 0):.1f}s")
        
        if all_summaries:
            print(f"\n� 观察历程：")
            for i, (summary, control) in enumerate(zip(all_summaries, all_controls), 1):
                print(f"   {i}. {summary} → {control}")
        
        print(f"{'='*60}\n")
        
        # 保存总结到文件
        if self.output_dir:
            summary_path = os.path.join(self.output_dir, "summary.txt")
            with open(summary_path, "w", encoding="utf-8") as f:
                f.write(f"探索总结\n")
                f.write(f"{'='*60}\n")
                f.write(f"总观察次数: {len(self.history)}\n\n")
                f.write(f"观察历程：\n")
                for i, (summary, control) in enumerate(zip(all_summaries, all_controls), 1):
                    f.write(f"{i}. {summary} → {control}\n")
            
            print(f"📁 总结已保存：{summary_path}")


# ======== 主函数 ========
def main():
    print(f"\n{'='*70}")
    print(f"🤖 Qwen 视觉控制系统 - 闭环交互版")
    print(f"{'='*70}\n")
    
    # 1. 检查模拟器
    simulator = SimulatorInterface()
    
    print("1️⃣ 检查模拟器状态...")
    if not simulator.is_ready():
        print("❌ 模拟器未运行！")
        print("\n请先在另一个终端运行：")
        print("   python 'pybullt/ test_environment.py'")
        print("\n等待演示动画结束后（约3秒），再运行本脚本。\n")
        return
    print("✅ 模拟器已就绪\n")
    
    # 2. 初始化 Qwen 分析器
    print("2️⃣ 初始化 Qwen-VL 分析器...")
    analyzer = QwenVisionAnalyzer(MODEL_NAME)
    print("✅ 分析器已就绪\n")
    
    # 3. 提示任务
    print("3️⃣ 探索模式：")
    print("   - Qwen 会描述看到的内容")
    print("   - Qwen 自主决定下一步动作")
    print("   - 通过旋转、移动来探索环境")
    print("   - 满足好奇心，观察有趣的物体")
    print("   - Qwen 觉得看够了会自己结束\n")
    
    # 4. 等待用户确认
    try:
        user_input = input("🚀 按 Enter 开始视觉控制任务，或输入 'n' 取消: ").strip().lower()
        if user_input == 'n':
            print("❌ 已取消任务")
            return
    except KeyboardInterrupt:
        print("\n❌ 已取消任务")
        return
    
    # 5. 运行视觉控制循环
    controller = VisionControlLoop(simulator, analyzer)
    success = controller.run()
    
    if success:
        print("\n✅ 视觉控制任务成功完成！")
    else:
        print("\n⚠️ 视觉控制任务未完全完成，但已尽力尝试。")
    
    print(f"\n💾 所有数据已保存到：{controller.output_dir}")
    print(f"   共完成 {len(controller.history)} 次迭代\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️ 任务被用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误：{e}")
        import traceback
        traceback.print_exc()
