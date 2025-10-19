#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速测试 Qwen 分析功能（单次测试）
"""

import os
import sys
import json
import re
import subprocess
from PIL import Image

MODEL_NAME = "lmstudio-community/Qwen3-VL-8B-Instruct-MLX-8bit"
TEST_IMAGE = "/Users/wjx_macair/Desktop/code/TEST/QWEN-OUTPUT/run_20251019_000652/iteration_01/camera_view.png"

VISION_SYSTEM_PROMPT = """你是一个装在机械臂末端的摄像头AI控制器。

**重要说明：这是一个连续控制任务！**

# 机械臂控制说明：

## 可用控制键（每个键的速度为 120°/秒）：
- **I键**：J1底座逆时针旋转（摄像头向左转）
- **K键**：J1底座顺时针旋转（摄像头向右转）
- **J键**：J2肩部抬起（摄像头向上看）
- **L键**：J2肩部下降（摄像头向下看）
- **U键**：J3肘部向上（微调视角）
- **O键**：J3肘部向下（微调视角）
- **Q键**：轨道逆时针移动（绕环境走动）
- **P键**：轨道顺时针移动（绕环境走动）
- **Y键**：升降上升（摄像头升高）
- **H键**：升降下降（摄像头降低）

## 控制时长（duration）：
- 计算公式：duration(秒) × 120°/秒 = 旋转角度
- 推荐：0.15~0.5秒（即18°~60°）
- 初次搜索：可用0.5~1.0秒大幅旋转
- 精细调整：用0.15~0.3秒小步调整

# 输出格式（JSON，自由发挥）：
```json
{
  "summary": "一句话总结你看到的内容（例如：看到绿色植物，有多片叶子）",
  "control": {
    "key": "K",
    "duration": 0.3
  }
}
```

只输出JSON，不要其他文字。"""


def parse_json_response(output: str) -> dict:
    """解析 Qwen 输出中的 JSON"""
    # 提取 assistant 回复部分
    assistant_match = re.search(r'<\|im_start\|>assistant\s*(.*?)(?:=====|$)', output, flags=re.DOTALL)
    if assistant_match:
        output = assistant_match.group(1)
    
    # 去除代码块标记
    cleaned = re.sub(r"```(?:json)?", "", output, flags=re.IGNORECASE).strip()
    
    # 尝试提取 JSON 对象
    json_matches = re.findall(r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}', cleaned, flags=re.DOTALL)
    
    # 从后往前尝试（最后一个JSON通常是实际输出）
    for candidate in reversed(json_matches):
        try:
            data = json.loads(candidate)
            # 验证必需字段
            if "summary" in data and "control" in data:
                summary = data.get("summary", "")
                # 排除提示词中的示例
                if "一句话总结" not in summary and "示例" not in summary:
                    return data
        except json.JSONDecodeError:
            continue
    
    return None


def test_single_analysis():
    """测试单次Qwen分析"""
    print(f"\n{'='*70}")
    print(f"🧪 测试 Qwen 分析功能")
    print(f"{'='*70}\n")
    
    if not os.path.exists(TEST_IMAGE):
        print(f"❌ 测试图像不存在：{TEST_IMAGE}\n")
        return
    
    print(f"📸 测试图像：{TEST_IMAGE}")
    
    img = Image.open(TEST_IMAGE)
    print(f"   尺寸：{img.size}")
    print(f"   模式：{img.mode}\n")
    
    # 构建提示词
    history_context = "\n\n# 这是第一次观察\n请描述你看到的内容，并决定第一步探索动作。"
    prompt = f"{VISION_SYSTEM_PROMPT}{history_context}\n\n请分析当前图像并输出控制指令："
    
    # 调用模型
    conda_python = "/Users/wjx_macair/miniforge/envs/bullet312/bin/python"
    python_cmd = conda_python if os.path.exists(conda_python) else sys.executable
    
    cmd = [
        python_cmd, "-m", "mlx_vlm.generate",
        "--model", MODEL_NAME,
        "--prompt", prompt,
        "--image", TEST_IMAGE,
        "--max-tokens", "600",
        "--temp", "0.3"
    ]
    
    print(f"🤖 调用 Qwen-VL...\n")
    
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=90
        )
        
        if result.returncode != 0:
            print(f"❌ 调用失败：{result.stderr}\n")
            return
        
        print(f"✅ 调用成功！\n")
        print(f"{'─'*70}")
        print(f"原始输出：")
        print(f"{'─'*70}\n")
        print(result.stdout)
        print(f"\n{'─'*70}\n")
        
        # 解析JSON
        parsed = parse_json_response(result.stdout)
        
        if parsed:
            print(f"✅ 成功解析！\n")
            print(f"{'─'*70}")
            print(f"解析结果：")
            print(f"{'─'*70}\n")
            print(json.dumps(parsed, indent=2, ensure_ascii=False))
            
            summary = parsed.get("summary", "")
            control = parsed.get("control", {})
            key = control.get("key", "?")
            duration = control.get("duration", 0)
            
            print(f"\n{'─'*70}")
            print(f"📊 分析：")
            print(f"{'─'*70}")
            print(f"   💬 总结：{summary}")
            print(f"   🎮 控制：{key} 键 {duration:.1f}s")
            print(f"{'─'*70}\n")
        else:
            print(f"❌ 无法解析 JSON\n")
    
    except Exception as e:
        print(f"❌ 错误：{e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_single_analysis()
