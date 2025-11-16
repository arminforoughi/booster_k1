# Booster K1 - Smart Robot System

## Quick Start

### SSH into robot
```bash
ssh booster@192.168.88.153
```

## Available Programs

### 1. Basic Robot Controls
```bash
python src/basic_controls.py eth0
```
- Keyboard control (WASD + QE for rotation)
- Hand gestures (rock, paper, scissors, grasp, OK)
- Head control (up, down, left, right)
- Voice commands (optional)

### 2. Basic Camera Feed
```bash
python src/basic_cam.py
```
- Web-based camera viewer
- View at: http://192.168.88.153:8080
- Supports stereo cameras with `--stereo` flag

### 3. YOLO Object Detection
```bash
python src/came_yolo.py
```
- Real-time object/face detection
- Web interface at: http://192.168.88.153:8080
- Options: `--model yolov8n`, `--detection face/object`

### 4. Smart Recognition System with Voice Learning (NEW! 🎯)
```bash
python src/smart_recognition.py
```

**Features:**
- YOLO object detection
- Face recognition with DeepFace
- Text-to-Speech (asks "Who is this?" for unknown people)
- **Voice learning - K1 listens to your answer and learns names!**
- Object naming database
- Web interface at: http://192.168.88.153:8080

**Usage:**
1. When K1 sees an unknown person, it asks "Who is this?"
2. **Say your name out loud** - K1 listens and learns automatically!
3. Alternatively, use web interface to teach names
4. Name objects through the web UI
5. K1 remembers everyone and everything!

**Disable voice learning (use web only):**
```bash
python src/smart_recognition.py --no-voice
```

**Installation:**
```bash
# Install Python dependencies
pip install -r requirements.txt

# Install TTS (choose one):

# Option A: Piper TTS (RECOMMENDED - neural voice, ARM-optimized)
bash install_piper.sh

# Option B: espeak (fallback - robotic but always works)
sudo apt-get install espeak

# Note: System auto-detects and uses best available TTS
```

## Features

- ✅ Robot locomotion control
- ✅ Camera feed streaming
- ✅ YOLO object detection
- ✅ Face recognition with DeepFace
- ✅ Text-to-Speech interaction
- ✅ Voice recognition and learning
- ✅ Person/object database
- ✅ Auto-learning mode

See [ROADMAP.md](ROADMAP.md) for planned features!
