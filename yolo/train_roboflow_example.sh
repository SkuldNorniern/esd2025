#!/bin/bash
# Example script to train YOLO model using Roboflow tennis ball dataset
# https://universe.roboflow.com/homework2warnerhoefakker/tennis-kfdyt

# Set your Roboflow API key (get it from https://app.roboflow.com/)
export ROBOFLOW_API_KEY="aL8OdaRliKhzoBjq2B6t"

# Train using Roboflow URL (easiest method)
uv run  main.py \
    --roboflow \
    --roboflow-url "https://universe.roboflow.com/homework2warnerhoefakker/tennis-kfdyt" \
    --model yolo11n.pt \
    --epochs 100 \
    --batch 16 \
    --imgsz 640 \
    --device 0,1 \
    --name tennis_ball_tracker \
    --loggers tensorboard

# Alternative: Train using workspace and project names
# python main.py \
#     --roboflow \
#     --roboflow-workspace "homework2warnerhoefakker" \
#     --roboflow-project "tennis-kfdyt" \
#     --model yolo11n.pt \
#     --epochs 100 \
#     --batch 16 \
#     --imgsz 640 \
#     --device 0 \
#     --name tennis_ball_tracker \
#     --loggers tensorboard

