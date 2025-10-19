#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
诊断 Qwen 分析失败问题
"""

import os
import sys
import json
import subprocess
from PIL import Image

MODEL_NAME = "lmstudio-community/Qwen3-VL-8B-Instruct-MLX-8bit"
TEST_IMAGE = "/Users/wjx_macair/Desktop/code/TEST/QWEN-OUTPUT/run_20251019_000652/iteration_01/camera_view.png"

# 简化的提示词（用于测试）
SIMPLE_PROMPT = """你是一个装在机械臂末端的摄像头AI控制器。

请分析图像并输出JSON格式的控制指令：

```json
{
  "summary": "一句话总结你看到的内容",
  "control": {
    "key": "K",
    "duration": 0.3
  }
}
```

只输出JSON，不要其他文字。"""


def test_qwen_call():
    """测试Qwen模型调用"""
    print(f"\n{'='*60}")
    print(f"🔍 诊断 Qwen 模型调用")
    print(f"{'='*60}\n")
    
    # 检查图像是否存在
    if not os.path.exists(TEST_IMAGE):
        print(f"❌ 测试图像不存在：{TEST_IMAGE}")
        print("\n请先运行 qwen-vision-control.py 生成测试图像\n")
        return
    
    print(f"✅ 找到测试图像：{TEST_IMAGE}")
    
    # 检查图像是否有效
    try:
        img = Image.open(TEST_IMAGE)
        print(f"✅ 图像尺寸：{img.size}")
        print(f"✅ 图像模式：{img.mode}\n")
    except Exception as e:
        print(f"❌ 图像无效：{e}\n")
        return
    
    # 测试Qwen调用
    print(f"🤖 调用 Qwen 模型...")
    print(f"   模型：{MODEL_NAME}")
    print(f"   图像：{TEST_IMAGE}\n")
    
    # 使用 conda 环境中的 Python
    conda_python = "/Users/wjx_macair/miniforge/envs/bullet312/bin/python"
    python_cmd = conda_python if os.path.exists(conda_python) else sys.executable
    
    cmd = [
        python_cmd, "-m", "mlx_vlm.generate",
        "--model", MODEL_NAME,
        "--prompt", SIMPLE_PROMPT,
        "--image", TEST_IMAGE,
        "--max-tokens", "300",
        "--temp", "0.3"
    ]
    
    print(f"使用 Python: {python_cmd}")
    print(f"命令：{' '.join(cmd)}\n")
    print(f"{'─'*60}\n")
    
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=90
        )
        
        print(f"返回码：{result.returncode}\n")
        
        if result.returncode != 0:
            print(f"❌ 调用失败！\n")
            print(f"stderr 输出：\n{result.stderr}\n")
            return
        
        print(f"✅ 调用成功！\n")
        print(f"{'─'*60}")
        print(f"📝 Qwen 原始输出：")
        print(f"{'─'*60}\n")
        print(result.stdout)
        print(f"\n{'─'*60}\n")
        
        # 尝试解析JSON
        print(f"🔍 尝试解析 JSON...\n")
        
        import re
        output = result.stdout.strip()
        
        # 去除代码块标记
        cleaned = re.sub(r"```(?:json)?", "", output, flags=re.IGNORECASE).strip()
        print(f"清理后的文本：\n{cleaned}\n")
        
        # 尝试提取 JSON 对象
        json_matches = re.findall(r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}', cleaned, flags=re.DOTALL)
        
        print(f"找到 {len(json_matches)} 个 JSON 候选\n")
        
        for i, candidate in enumerate(json_matches, 1):
            print(f"候选 {i}：")
            print(candidate)
            print()
            
            try:
                data = json.loads(candidate)
                print(f"✅ 成功解析！")
                print(json.dumps(data, indent=2, ensure_ascii=False))
                
                # 检查必需字段
                if "summary" in data and "control" in data:
                    print(f"\n✅ 包含必需字段 (summary, control)")
                else:
                    print(f"\n⚠️ 缺少必需字段")
                    print(f"   当前字段：{list(data.keys())}")
                
                print()
            except json.JSONDecodeError as e:
                print(f"❌ JSON 解析失败：{e}\n")
        
    except subprocess.TimeoutExpired:
        print("❌ 调用超时（90秒）")
    except FileNotFoundError:
        print("❌ 找不到 mlx_vlm")
        print("\n请安装：pip install mlx-vlm\n")
    except Exception as e:
        print(f"❌ 发生错误：{e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_qwen_call()
