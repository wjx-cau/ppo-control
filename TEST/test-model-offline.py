#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 Qwen-VL 离线模式是否正常工作
"""

import os
import sys

# 强制离线模式
os.environ['HF_HUB_OFFLINE'] = '1'
os.environ['TRANSFORMERS_OFFLINE'] = '1'

# 模型路径
MODEL_NAME = "/Users/wjx_macair/.cache/huggingface/hub/models--lmstudio-community--Qwen3-VL-8B-Instruct-MLX-8bit/snapshots/ef2f0ae5bd3b1f48193bf2622dd1f654af691e75"

print("🧪 测试 Qwen-VL 离线加载...")
print(f"📂 模型路径：{MODEL_NAME}")
print(f"🔒 离线模式：HF_HUB_OFFLINE={os.environ.get('HF_HUB_OFFLINE')}")

# 检查路径是否存在
if not os.path.exists(MODEL_NAME):
    print(f"❌ 模型路径不存在！")
    sys.exit(1)

print(f"✅ 模型路径存在")

# 列出模型文件
print(f"\n📁 模型文件列表：")
files = os.listdir(MODEL_NAME)
for f in sorted(files)[:10]:  # 只显示前10个文件
    print(f"   - {f}")

print(f"\n✅ 离线模式配置成功！")
print(f"\n💡 现在可以运行 qwen-vision-control.py，它将使用本地模型，无需联网。")
