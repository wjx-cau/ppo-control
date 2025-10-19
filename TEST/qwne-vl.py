#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 VS Code 中使用 Qwen3-VL-8B-Instruct-MLX-8bit，
自动标注图片中“红色”物体并保存标注结果。
"""

# ======== 路径配置（请改成你自己的） ========
IMAGE_PATH = "截屏2025-10-16 下午11.16.47.png"     # 要分析的图片
OUT_PATH   = "annotated.png" # 输出图片
REPORT_PATH = "report.txt"   # 输出文字报告
MODEL_NAME = "lmstudio-community/Qwen3-VL-8B-Instruct-MLX-8bit"  # 模型名称
# ============================================

import os, re, json, subprocess, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import cv2

# ---------- 提示词 ----------
PROMPT = (
    "你是一名图像标注助手。"
    "任务：仅针对图像中‘红色’的物体进行框选，输出每个红色目标的边界框和名称。"
    "请严格返回一个 JSON 数组，不要包含多余文字。"
    "数组中的每个元素是一个对象，包含字段："
    '{"label": "类别或名称(如 red-shirt, red-car 等)", "bbox": [x1, y1, x2, y2]}。'
    "坐标是整数像素，x1,y1 为左上角，x2,y2 为右下角。"
    "只允许出现这个 JSON 数组，别输出其他文本。"
)

# ---------- 调用 Qwen ----------
def run_qwen(image_path, model, max_tokens=512):
    """运行 mlx_vlm.generate 调用 Qwen 模型"""
    cmd = [
        sys.executable, "-m", "mlx_vlm.generate",
        "--model", model,
        "--prompt", PROMPT,
        "--image", image_path,
        "--max-tokens", str(max_tokens)
    ]
    print(f"Running: {' '.join(cmd)}\n")
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, check=False)
        output = res.stdout.strip()
    except FileNotFoundError:
        print("❌ 找不到 mlx_vlm.generate，请先安装：pip install -U mlx-vlm")
        return "", None

    # 尝试解析 JSON 数组
    cleaned = re.sub(r"```(?:json)?", "", output, flags=re.IGNORECASE).strip()
    json_array = None
    candidates = re.findall(r"\[.*\]", cleaned, flags=re.DOTALL)
    for cand in candidates:
        try:
            data = json.loads(cand)
            if isinstance(data, list):
                json_array = data
                break
        except Exception:
            continue
    return output, json_array

# ---------- HSV 颜色检测备用 ----------
def fallback_detect_red(image_path):
    """经典 HSV 红色检测"""
    img_bgr = cv2.imread(image_path)
    if img_bgr is None:
        return []
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0, 100, 80])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 100, 80])
    upper2 = np.array([180, 255, 255])
    mask = cv2.inRange(img_hsv, lower1, upper1) | cv2.inRange(img_hsv, lower2, upper2)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w*h < 200:
            continue
        boxes.append({"label": "red", "bbox": [int(x), int(y), int(x+w), int(y+h)]})
    return boxes

# ---------- 绘制框 ----------
def draw_boxes(image, boxes):
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("Arial Unicode.ttf", 16)
    except Exception:
        font = ImageFont.load_default()

    for obj in boxes:
        bbox = obj.get("bbox", [])
        label = obj.get("label", "red-object")
        if len(bbox) != 4:
            continue
        x1, y1, x2, y2 = map(int, bbox)
        draw.rectangle([x1, y1, x2, y2], outline=(255, 0, 0), width=3)
        draw.rectangle([x1, y1-20, x1+len(label)*8+6, y1], fill=(255,0,0))
        draw.text((x1+3, y1-18), label, fill=(255,255,255), font=font)
    return image

# ---------- 主流程 ----------
def main():
    if not os.path.exists(IMAGE_PATH):
        print(f"❌ 找不到图片：{IMAGE_PATH}")
        return

    # 1. 调用模型
    model_output, boxes = run_qwen(IMAGE_PATH, MODEL_NAME)

    # 2. 如果没返回 JSON，则用 HSV 兜底
    fallback_used = False
    if not boxes:
        boxes = fallback_detect_red(IMAGE_PATH)
        fallback_used = True

    # 3. 绘制结果
    img = Image.open(IMAGE_PATH).convert("RGB")
    img_annot = draw_boxes(img, boxes)
    img_annot.save(OUT_PATH)
    print(f"✅ 已保存标注图片：{OUT_PATH}")

    # 4. 输出报告
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write("# Qwen Red Annotation Report\n")
        f.write(f"image: {IMAGE_PATH}\n")
        f.write(f"model: {MODEL_NAME}\n")
        f.write(f"fallback_used: {fallback_used}\n\n")
        f.write("== model_output ==\n")
        f.write(model_output + "\n\n")
        f.write("== parsed_boxes ==\n")
        f.write(json.dumps(boxes, ensure_ascii=False, indent=2))
    print(f"📝 报告已保存：{REPORT_PATH}")
    if fallback_used:
        print("⚠️ 模型未返回有效 JSON，使用了 HSV 颜色检测备用结果。")

if __name__ == "__main__":
    main()
