# Booster K1 Development Roadmap

Building a simple, working robotics intelligence system inspired by the whoami project.

---

## Current Working Features ✅

### Vision System
- [x] **Basic Camera Feed** (`src/basic_cam.py`)
  - ROS2 camera subscriber
  - Web-based viewer (HTTP server)
  - Stereo camera support (left/right)
  - NV12 format handling

- [x] **YOLO Detection** (`src/came_yolo.py`)
  - YOLOv8 object detection
  - Face detection (OpenCV fallback)
  - Real-time web feed with annotations
  - FPS counter

### Robot Control
- [x] **Basic Controls** (`src/basic_controls.py`)
  - Keyboard control (WASD + QE rotation)
  - Dynamic movement (continuous key press)
  - Hand gestures (rock, paper, scissors, grasp, OK)
  - Head control (up, down, left, right)
  - Mode switching (Prepare, Walking, Damping, Custom)
  - Voice command support (optional)

---

## Roadmap: Features from whoami

### Phase 1: Core Intelligence 🎯
**Goal: Add face recognition and basic person tracking**

- [ ] **Face Recognition Engine**
  - [ ] DeepFace integration
  - [ ] Face embedding storage
  - [ ] Person identification
  - [ ] Confidence scoring
  - [ ] Multiple face handling

- [ ] **Person Database**
  - [ ] Simple JSON storage (start simple)
  - [ ] Store: name, face embeddings, timestamps
  - [ ] CRUD operations (Create, Read, Update, Delete)
  - [ ] Later: Consider Gun.js encrypted storage

- [ ] **Name Learning**
  - [ ] Detect unknown faces
  - [ ] Prompt for name input
  - [ ] Associate name with face embedding
  - [ ] Remember across sessions

### Phase 2: Voice Interaction 🎤
**Goal: Natural conversation and responses**

- [ ] **Voice Input** (partially done)
  - [x] Speech recognition (SpeechRecognition)
  - [ ] Better wake word detection
  - [ ] Noise filtering
  - [ ] Command vs conversation mode

- [ ] **Voice Output**
  - [ ] Text-to-Speech (pyttsx3 or espeak)
  - [ ] F5-TTS neural voice (advanced)
  - [ ] Personality/tone settings
  - [ ] Response queue management

- [ ] **Conversation Logic**
  - [ ] Greet recognized people by name
  - [ ] Respond to questions
  - [ ] Remember conversation context
  - [ ] Integrate with robot actions

### Phase 3: Integration 🔗
**Goal: Connect vision, voice, and control**

- [ ] **Unified Control System**
  - [ ] Vision triggers actions (e.g., wave when person detected)
  - [ ] Voice controls movement
  - [ ] Face recognition triggers greetings
  - [ ] Autonomous behavior modes

- [ ] **Behavior System**
  - [ ] Idle behavior (look around, wave)
  - [ ] Approach detected person
  - [ ] Follow known person
  - [ ] Avoid obstacles

- [ ] **Web Dashboard**
  - [ ] Combined camera + detection feed
  - [ ] Show recognized people
  - [ ] Control panel for robot
  - [ ] Status indicators
  - [ ] Database viewer/editor

### Phase 4: Hardware Abstraction 🔧
**Goal: Make it work on different platforms**

- [ ] **Platform Detection**
  - [ ] Auto-detect Jetson vs other platforms
  - [ ] GPU availability check
  - [ ] Camera capability detection
  - [ ] Network interface auto-config

- [ ] **Fallback Systems**
  - [ ] CPU-only mode when no GPU
  - [ ] USB camera fallback
  - [ ] Reduced model sizes for lower-end hardware

### Phase 5: Advanced Features 🚀
**Goal: Enhanced capabilities**

- [ ] **Object Tracking**
  - [ ] Track specific objects
  - [ ] Hand object to person
  - [ ] Remember object locations

- [ ] **Gesture Recognition**
  - [ ] Recognize hand gestures from camera
  - [ ] Respond to pointing
  - [ ] Wave back when someone waves

- [ ] **Emotion Detection**
  - [ ] Detect facial expressions
  - [ ] Adjust behavior based on mood
  - [ ] Empathetic responses

- [ ] **Multi-Robot Communication**
  - [ ] Share knowledge between robots
  - [ ] Coordinated tasks
  - [ ] Distributed database

---

## Current Sprint 🏃

**Not yet started - waiting for direction**

### Next Steps
1. Choose Phase 1, 2, or 3 to start
2. Break down first task
3. Implement incrementally
4. Test on real hardware
5. Update this roadmap

---

## Technical Stack

### Current Dependencies
- ROS2 (Humble/Foxy)
- OpenCV
- cv_bridge
- ultralytics (YOLOv8)
- booster_robotics_sdk_python
- SpeechRecognition (optional)
- pyaudio (optional)

### Planned Dependencies
- DeepFace (face recognition)
- pyttsx3 or F5-TTS (text-to-speech)
- NumPy, Pillow (image processing)
- Flask or FastAPI (better web server)
- SQLite or Gun.js (database)

---

## Development Philosophy

1. **Keep it simple** - Start with basic working version
2. **Incremental progress** - Add features one at a time
3. **Test on hardware** - Verify each feature on real robot
4. **Graceful degradation** - Fallbacks when dependencies missing
5. **Clear code** - Easy to understand and modify

---

## Notes

- whoami repo is the ROADMAP, not the codebase to copy
- We build features incrementally here in booster_k1
- Test each feature before moving to next
- Document what works and what doesn't
- Keep it simple and maintainable

---

**Last Updated:** 2025-11-16
