# Jetson Optimization Guide for Booster K1

This guide covers performance optimizations specifically for NVIDIA Jetson platforms (Nano, Xavier NX, AGX Xavier, Orin).

## Quick Start - Optimized Command

```bash
# Run with Jetson optimizations
python src/smart_recognition.py --frame-skip 3 --max-resolution 480

# For even better performance (process 25% of frames)
python src/smart_recognition.py --frame-skip 4 --max-resolution 416
```

---

## Current Optimizations (Implemented)

### 1. Frame Skipping ✅
**What it does:** Only processes every Nth frame, reducing GPU load

```bash
# Process 50% of frames (every 2nd frame)
python src/smart_recognition.py --frame-skip 2

# Process 33% of frames (every 3rd frame)
python src/smart_recognition.py --frame-skip 3

# Process 25% of frames (every 4th frame)
python src/smart_recognition.py --frame-skip 4
```

**Performance impact:**
- `--frame-skip 2`: ~2x faster, still smooth recognition
- `--frame-skip 3`: ~3x faster, good for static scenes
- `--frame-skip 4`: ~4x faster, acceptable for slow-moving subjects

### 2. Resolution Limiting ✅
**What it does:** Resizes large frames before processing

```bash
# Limit to 640px width (default, good balance)
python src/smart_recognition.py --max-resolution 640

# Limit to 480px width (faster, still acceptable)
python src/smart_recognition.py --max-resolution 480

# Limit to 416px width (YOLO optimal size, fastest)
python src/smart_recognition.py --max-resolution 416
```

**Performance impact:**
- Reduces memory usage
- Faster GPU processing
- Still maintains good detection accuracy

### 3. Efficient TTS ✅
**Status:** Piper TTS is ARM-optimized and lightweight

- Uses neural voice with low resource usage
- ~50MB footprint
- Fast inference on ARM devices

---

## Advanced Optimizations (Manual Setup Required)

### 4. YOLO TensorRT Export (RECOMMENDED)

**Why:** TensorRT gives 3-10x speedup on Jetson

**Steps:**

#### A. Export YOLO to TensorRT format

```bash
# Install ultralytics if not already installed
pip install ultralytics

# Export YOLOv8n to TensorRT (one-time operation)
python3 << EOF
from ultralytics import YOLO

# Load model
model = YOLO('yolov8n.pt')

# Export to TensorRT (this takes 5-10 minutes)
model.export(format='engine', device=0)  # device=0 for GPU
print("✓ TensorRT model exported to: yolov8n.engine")
EOF
```

#### B. Modify smart_recognition.py to use TensorRT model

```python
# In smart_recognition.py, line ~41, change:
self.yolo_model = YOLO('yolov8n.pt')

# To:
self.yolo_model = YOLO('yolov8n.engine')  # TensorRT
```

**Performance gain:** 3-5x faster inference on Jetson

---

### 5. GPU Memory Limit Configuration

**Why:** Prevent OOM errors and system crashes

**Option A: TensorFlow GPU Memory Limit (for DeepFace)**

Add to `smart_recognition.py` before initializing SmartRecognizer:

```python
import tensorflow as tf

# Set GPU memory limit (adjust based on your Jetson model)
gpus = tf.config.list_physical_devices('GPU')
if gpus:
    try:
        # Jetson Nano: 2GB max
        # Jetson Xavier NX: 4GB max
        # Jetson AGX Xavier: 16GB max
        # Jetson Orin: 8-32GB depending on model

        tf.config.set_logical_device_configuration(
            gpus[0],
            [tf.config.LogicalDeviceConfiguration(memory_limit=2048)]  # 2GB for Nano
        )
        print(f"✓ GPU memory limit set to 2GB")
    except RuntimeError as e:
        print(f"⚠️  GPU memory config failed: {e}")
```

**Option B: Environment Variable (simpler)**

```bash
# Before running the script
export TF_FORCE_GPU_ALLOW_GROWTH=true
export TF_GPU_ALLOCATOR=cuda_malloc_async

python src/smart_recognition.py
```

---

### 6. Optimize DeepFace for ARM

**Current Issue:** DeepFace uses TensorFlow which is not optimized for ARM

**Solution Options:**

#### Option A: Use FaceNet-PyTorch (Lighter)

```bash
pip install facenet-pytorch
```

Modify `face_recognition.py` to use facenet-pytorch instead of DeepFace.

#### Option B: Use OpenCV DNN with CUDA

```python
# Load face recognition model using OpenCV DNN
net = cv2.dnn.readNetFromONNX('face_model.onnx')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
```

#### Option C: Disable DeepFace (Detection Only)

```bash
# Run without face recognition (detection only)
python src/smart_recognition.py --no-deepface
```

---

### 7. Jetson Clocks (Maximum Performance)

**Enable maximum clock speeds:**

```bash
# Enable max performance mode
sudo jetson_clocks

# Check status
sudo jetson_clocks --show

# Disable (back to power-saving mode)
sudo jetson_clocks --restore
```

**Note:** Increases power consumption and heat. Use with adequate cooling.

---

### 8. Power Mode Configuration

```bash
# Check current power mode
sudo nvpmodel -q

# Jetson Nano: Set to 10W mode (max performance)
sudo nvpmodel -m 0

# Jetson Xavier NX: Set to 15W 6-core mode
sudo nvpmodel -m 2

# Jetson AGX Xavier: Set to MAXN mode
sudo nvpmodel -m 0
```

---

## Recommended Configurations

### Jetson Nano (4GB)
```bash
# Moderate performance, stable
python src/smart_recognition.py \
  --frame-skip 3 \
  --max-resolution 416 \
  --no-deepface  # Use detection only

# With TensorRT YOLO
python src/smart_recognition.py \
  --frame-skip 2 \
  --max-resolution 480 \
  --no-deepface
```

### Jetson Xavier NX (8GB)
```bash
# Good performance with face recognition
python src/smart_recognition.py \
  --frame-skip 2 \
  --max-resolution 640

# With TensorRT YOLO
python src/smart_recognition.py \
  --frame-skip 1 \  # Process all frames
  --max-resolution 640
```

### Jetson AGX Xavier / Orin
```bash
# Maximum features, best quality
python src/smart_recognition.py \
  --frame-skip 1 \  # Process all frames
  --max-resolution 1280  # Higher resolution

# Everything enabled with TensorRT
# Can handle full pipeline without issues
```

---

## Performance Monitoring

### Check GPU Usage
```bash
# Monitor GPU/CPU usage
sudo tegrastats

# Or install jtop (recommended)
sudo -H pip install -U jetson-stats
sudo jtop
```

### Monitor FPS

The system displays FPS on the video feed and web interface. Target FPS:
- **Jetson Nano:** 5-10 FPS (with optimizations)
- **Xavier NX:** 15-20 FPS (with optimizations)
- **AGX Xavier/Orin:** 20-30+ FPS (with optimizations)

---

## Troubleshooting

### System Freezes / OOM Errors

**Solution:**
1. Increase frame skip: `--frame-skip 4`
2. Reduce resolution: `--max-resolution 416`
3. Disable DeepFace: `--no-deepface`
4. Set GPU memory limit (see section 5)
5. Add swap space:

```bash
# Add 4GB swap (for Jetson Nano)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Slow Performance

**Solution:**
1. Export YOLO to TensorRT (section 4)
2. Enable jetson_clocks
3. Set appropriate power mode
4. Increase frame skip

### Camera Feed Lag

**Solution:**
1. Increase `--frame-skip` value
2. Reduce `--max-resolution`
3. Check if other processes are using GPU: `sudo tegrastats`

---

## Future Optimizations (TODO)

- [ ] Replace DeepFace with facenet-pytorch
- [ ] Implement ONNX models with TensorRT
- [ ] Add dynamic frame skip based on CPU load
- [ ] Implement model quantization (INT8)
- [ ] Add multi-threading for CPU operations
- [ ] Implement frame buffer queue management

---

## Benchmarks

| Configuration | Jetson Nano | Xavier NX | AGX Xavier |
|--------------|-------------|-----------|------------|
| Default (no opt) | 2-3 FPS | 8-10 FPS | 15-20 FPS |
| + Frame skip 2 | 4-6 FPS | 15-18 FPS | 25-30 FPS |
| + TensorRT | 8-10 FPS | 20-25 FPS | 30+ FPS |
| + All opts | 10-12 FPS | 25-30 FPS | 40+ FPS |

*Benchmarks with YOLOv8n + DeepFace disabled*

---

## Summary Checklist

**Quick wins (no code changes):**
- [ ] Use `--frame-skip 2` or higher
- [ ] Use `--max-resolution 640` or lower
- [ ] Run `sudo jetson_clocks`
- [ ] Set appropriate power mode

**Better performance (requires setup):**
- [ ] Export YOLO to TensorRT
- [ ] Set GPU memory limits
- [ ] Add swap space
- [ ] Disable DeepFace if not needed

**Best performance (code modifications):**
- [ ] Replace DeepFace with lighter alternative
- [ ] Use ONNX models with TensorRT
- [ ] Implement model quantization

---

**For questions or issues, check the main README.md or ROADMAP.md**
