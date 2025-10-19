#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试本地模型是否能在 MLX 环境下运行。
模型路径: /Users/wjx_macair/Qwen3-VL-8B-Instruct-FP8
"""

# ======== 路径与配置 ========
MODEL_PATH = "/Users/wjx_macair/Qwen3-VL-8B-Instruct-FP8"   # 模型文件夹路径
PROMPT = "你好，我是用户。请简单介绍一下你自己。"
# ===========================

import subprocess, sys, os

def main():
    print(f"🔧 正在调用本地模型：{MODEL_PATH}\n")

    # 检查路径
    if not os.path.exists(MODEL_PATH):
        print(f"❌ 模型路径不存在，请检查：{MODEL_PATH}")
        return

    # 调用 mlx_vlm.generate 运行推理
    cmd = [
        sys.executable, "-m", "mlx_vlm.generate",
        "--model", MODEL_PATH,
        "--prompt", PROMPT,
        "--max-tokens", "128"
    ]

    print("▶ 正在运行命令：")
    print(" ".join(cmd), "\n")

    try:
        result = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, check=False
        )
        print("✅ 模型输出：\n")
        print(result.stdout)

    except FileNotFoundError:
        print("❌ 未找到 mlx_vlm 工具，请先执行：pip install -U mlx-vlm")
    except Exception as e:
        print("❌ 出错：", e)

if __name__ == "__main__":
    main()
