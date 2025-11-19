#!/bin/bash
# Convert YOLO PyTorch models to TensorRT for Jetson optimization
# Provides 3-7x faster inference on NVIDIA Jetson Orin NX

echo "============================================================"
echo "🚀 YOLO to TensorRT Converter for Jetson"
echo "============================================================"
echo ""

# Check if running on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "⚠️  Warning: Not running on Jetson. TensorRT conversion may fail."
    echo "   This script is optimized for Jetson Orin NX."
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Models to convert
MODELS=("yolov8n" "yolov8s" "yolov8m")

echo "Available models to convert:"
for i in "${!MODELS[@]}"; do
    echo "$((i+1)). ${MODELS[$i]}"
done
echo ""

read -p "Select model number (1-${#MODELS[@]}) or press Enter for all: " choice

if [ -z "$choice" ]; then
    # Convert all models
    SELECTED_MODELS=("${MODELS[@]}")
else
    # Convert selected model
    SELECTED_MODELS=("${MODELS[$((choice-1))]}")
fi

# Check for FP16 support (all Jetson models support this)
USE_FP16="--half"

echo ""
echo "Converting models with settings:"
echo "  - Format: TensorRT"
echo "  - Precision: FP16 (half precision)"
echo "  - Device: GPU (device=0)"
echo ""

for model in "${SELECTED_MODELS[@]}"; do
    echo "============================================================"
    echo "Converting $model.pt → $model.engine"
    echo "============================================================"

    # Download model if not present
    if [ ! -f "${model}.pt" ]; then
        echo "Model not found. YOLO will download automatically..."
    fi

    # Convert using ultralytics YOLO export
    python3 << EOF
from ultralytics import YOLO

print(f"Loading {model}.pt...")
model = YOLO('${model}.pt')

print(f"Exporting to TensorRT...")
model.export(
    format='engine',
    device=0,
    half=True,  # FP16 precision for faster inference
    simplify=True,
    workspace=4,  # 4GB workspace for optimization
    verbose=True
)

print(f"✓ Conversion complete: ${model}.engine")
EOF

    if [ $? -eq 0 ]; then
        echo "✅ ${model}.engine created successfully"

        # Show file size
        if [ -f "${model}.engine" ]; then
            size=$(du -h "${model}.engine" | cut -f1)
            echo "   File size: $size"
        fi
    else
        echo "❌ Conversion failed for ${model}"
    fi
    echo ""
done

echo "============================================================"
echo "✅ CONVERSION COMPLETE"
echo "============================================================"
echo ""
echo "Usage examples:"
echo "  python src/came_yolo.py --tensorrt --model yolov8n"
echo "  python src/smart_recognition.py --tensorrt"
echo ""
echo "Performance tips:"
echo "  - TensorRT provides 3-7x faster inference on Jetson"
echo "  - FP16 precision has minimal accuracy impact"
echo "  - First run will be slower (engine optimization)"
echo "  - Subsequent runs will be much faster"
echo ""
echo "Note: .engine files are device-specific"
echo "      Do NOT copy to different Jetson models or Jetpack versions"
echo "============================================================"
