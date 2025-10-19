#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
查看摄像头捕获的图像
在 macOS Preview 中打开图像文件
"""

import os
import subprocess
from PIL import Image

CAMERA_DIR = "/tmp/pybullet_camera"
LATEST_IMAGE = "/tmp/pybullet_camera/latest.png"

def main():
    print("\n" + "="*60)
    print("📸 摄像头图像查看器")
    print("="*60 + "\n")
    
    # 检查目录
    if not os.path.exists(CAMERA_DIR):
        print(f"❌ 目录不存在：{CAMERA_DIR}\n")
        return
    
    # 列出所有图像
    images = [f for f in os.listdir(CAMERA_DIR) if f.endswith('.png')]
    
    if not images:
        print(f"❌ 没有找到图像文件\n")
        return
    
    print(f"✅ 找到 {len(images)} 张图像\n")
    
    # 显示最新图像信息
    if os.path.exists(LATEST_IMAGE):
        try:
            img = Image.open(LATEST_IMAGE)
            width, height = img.size
            print(f"📷 最新图像信息：")
            print(f"   路径: {LATEST_IMAGE}")
            print(f"   分辨率: {width}x{height}")
            print(f"   格式: {img.format}")
            print(f"   模式: {img.mode}")
            print(f"   大小: {os.path.getsize(LATEST_IMAGE)} 字节\n")
            
            # 检查是否是纯色图像
            extrema = img.getextrema()
            is_uniform = all(min_val == max_val for min_val, max_val in extrema)
            
            if is_uniform:
                print(f"⚠️ 警告：图像是纯色的（所有像素相同）")
                print(f"   这可能意味着摄像头未正确渲染\n")
            else:
                print(f"✅ 图像有内容变化\n")
            
        except Exception as e:
            print(f"❌ 无法读取图像：{e}\n")
    
    # 打开最新图像
    print("🖼️ 正在打开图像...\n")
    try:
        subprocess.run(["open", LATEST_IMAGE], check=True)
        print("✅ 图像已在 Preview 中打开\n")
    except Exception as e:
        print(f"❌ 打开失败：{e}\n")
    
    # 提供文件夹路径
    print(f"📂 所有图像路径：{CAMERA_DIR}")
    print(f"   使用命令查看所有图像：open {CAMERA_DIR}\n")

if __name__ == "__main__":
    main()
