#!/bin/bash
# Example script to train YOLO model using Roboflow tennis ball dataset
# https://universe.roboflow.com/homework2warnerhoefakker/tennis-kfdyt

# Set your Roboflow API key (get it from https://app.roboflow.com/)
# Or export it: export ROBOFLOW_API_KEY=your_api_key_here

uv run main.py \
    --roboflow \
    --roboflow-workspace homework2warnerhoefakker \
    --roboflow-project tennis \
    --roboflow-version latest \
    --model yolo11n.pt \
    --epochs 100 \
    --batch 16 \
    --imgsz 640 \
    --device 0 \
    --name tennis_ball_tracker \
    --loggers tensorboard

