# Booster K1 - Smart Robot System

## Quick Start

### SSH into robot
```bash
ssh booster@192.168.88.153
```

## Available Programs

### 1. Basic Robot Controls
```bash
python src/basic_controls.py
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

### 3. YOLO Object Detection with 3D Depth 🆕
```bash
python src/came_yolo.py
```
- Real-time object/face detection
- **3D position estimation using depth camera**
- **TensorRT support for 3-7x faster inference on Jetson**
- Web interface at: http://192.168.88.153:8080
- Options:
  - `--model yolov8n` - Model size (n/s/m/l/x)
  - `--detection face/object` - Detection type
  - `--tensorrt` - Use TensorRT engine (faster on Jetson)
  - `--depth` - Enable 3D depth (default: on)
  - `--no-depth` - Disable depth integration

**Convert models to TensorRT:**
```bash
./convert_yolo_to_tensorrt.sh
```

### 4. Wave Greeter Bot 👋 🆕
```bash
python src/wave_greeter.py
```
- **Detects hand waves** using MediaPipe or motion detection
- **Recognizes people** and remembers them
- **Personalized greetings** based on how many times you've waved
- **TTS responses** with context from past interactions
- Web interface at: http://192.168.88.153:8080

**Features:**
- First wave: "Hello John! Great to meet you!"
- 5th wave: "Hey John! You're becoming a regular!"
- 20th wave: "Hey John! You're one of my favorite people!"
- Unknown person: "Hello! I don't think we've met yet. I'm K1!"

**Install MediaPipe (optional, for better accuracy):**
```bash
pip install mediapipe
```

### 5. Smart Recognition System with Voice Learning 🎯
```bash
python src/smart_recognition.py
```

**Features:**
- YOLO object detection
- Face recognition with DeepFace
- Natural conversation with TTS (asks "Would you like to connect?")
- **Voice learning with conversation recording**
- Person/object database with conversation history
- Web interface at: http://192.168.88.153:8080

**Natural Conversation Flow:**
1. When K1 sees someone new, it asks "Would you like to connect?"
2. Say "yes" → K1 has a natural conversation:
   - "What's your name?"
   - "What brings you here today?"
   - Records the full conversation
3. Next time → K1 recognizes you and greets with context!
   - "Hey Sarah, good to see you!"
4. Privacy: Say "no" → K1 respects your privacy
5. Web interface available as backup for manual teaching

**Soft auto-start (ask permission before starting):**
```bash
python src/smart_recognition.py --auto-ask
```
K1 will ask "Would you like me to start?" and wait for your response!
- Say "yes" to start immediately
- Say "no" to enter standby mode (then say "start" when ready)

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

**Core Robotics:**
- ✅ Robot locomotion control (SDK via localhost)
- ✅ Camera feed streaming (ROS2)
- ✅ Emergency stop system with multi-level safety

**Computer Vision:**
- ✅ YOLO object detection with **3D depth integration** 🆕
- ✅ TensorRT optimization for Jetson (3-7x speedup) 🆕
- ✅ Face recognition with DeepFace
- ✅ Real-time depth mapping from ZED stereo camera

**Intelligence & Interaction:**
- ✅ **Wave detection greeter bot** 👋 🆕
- ✅ **Personalized greetings** based on interaction history 🆕
- ✅ Text-to-Speech interaction
- ✅ Voice recognition and learning
- ✅ Person/object database
- ✅ Auto-learning mode

**Fleet Operations:** 🆕
- ✅ ROS2 SDK bridge for standard `/cmd_vel` control
- ✅ Gun.js distributed fleet coordinator
- ✅ Multi-robot task distribution
- ✅ Decentralized state management

## Fleet Deployment 🤖

### ROS2 Bridge (Multi-Robot Control)
```bash
# Start ROS2 bridge for robot k1_001
python src/booster_ros2_bridge.py k1_001

# Control via standard ROS2 topics
ros2 topic pub /k1_001/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
ros2 topic pub /k1_001/mode_cmd std_msgs/String "data: walking"
```

### Fleet Coordinator (Task Distribution)
```bash
# Start fleet coordinator with Gun.js relay
python src/fleet_coordinator.py k1_001 http://gun-relay:8765

# Robots will automatically:
# - Publish heartbeats (battery, position, status)
# - Discover and claim tasks from shared queue
# - Monitor other robots in fleet
```

**Gun.js Database Structure:**
```
/fleet/robots/{robot_id}  - Heartbeat and status
/fleet/tasks/{task_id}    - Distributed task queue
```

See [ROADMAP.md](ROADMAP.md) for planned features!
