# 🚨 SAFETY CHECKLIST - FIRST K1 HARDWARE TEST

**STOP!** Before running ANY code on the K1 robot, complete this checklist.

---

## ⚠️ CRITICAL SAFETY MEASURES APPLIED

### ✅ Implemented Safety Systems

1. **Emergency Stop System**
   - ✅ Multiple stop commands sent on ANY exit scenario
   - ✅ Registered with `atexit` (handles normal exit, crashes, exceptions)
   - ✅ Signal handlers for SIGTERM and SIGINT (Ctrl+C, kill command)
   - ✅ Sends 10 stop commands with delays to ensure receipt
   - **Location:** `src/basic_controls.py:34-71`

2. **Non-Daemon Threads**
   - ✅ Control thread is non-daemon (ensures cleanup before exit)
   - ✅ Thread join with 2-second timeout
   - ✅ Warning if thread doesn't stop cleanly
   - **Location:** `src/basic_controls.py:116, 119-140`

3. **Atomic Database Writes**
   - ✅ Write to temp file first, then atomic move
   - ✅ Automatic backups before overwriting
   - ✅ Restore from backup on corruption
   - **Location:** `src/smart_recognition.py:103-148`

4. **Command Injection Protection**
   - ✅ TTS uses subprocess.run (NOT os.system)
   - ✅ Arguments passed as list (no shell interpretation)
   - ✅ Timeout protection (10 seconds)
   - **Location:** `src/tts_module.py:226-244`

---

## 📋 PRE-FLIGHT CHECKLIST

### Phase 1: Environment Setup

- [ ] **1.1** K1 robot is powered ON
- [ ] **1.2** K1 robot is in DAMPING mode (motors off)
- [ ] **1.3** Network interface is correct (usually `eth0`)
- [ ] **1.4** Computer and K1 are on same network
- [ ] **1.5** Can ping K1 robot: `ping <robot-ip>`
- [ ] **1.6** Physical emergency stop button is accessible
- [ ] **1.7** Clear space around robot (2+ meters)
- [ ] **1.8** Robot is on flat, stable surface

### Phase 2: Dependency Check

- [ ] **2.1** Python 3.8+ installed: `python3 --version`
- [ ] **2.2** All dependencies installed: `pip install -r requirements.txt`
- [ ] **2.3** Booster SDK installed and working
- [ ] **2.4** ROS2 (Humble/Foxy) installed and sourced
- [ ] **2.5** Camera topic exists: `ros2 topic list | grep camera`
- [ ] **2.6** Optional: Piper TTS installed: `bash install_piper.sh`

### Phase 3: Software Tests (Without Motors)

#### Test 1: Basic Connection
```bash
# Should connect and show robot mode
python src/basic_controls.py eth0
# Verify:
# - "✓ Emergency stop system active" message appears
# - "✓ Connection successful! Current robot mode: Damping"
# - Ctrl+C triggers emergency stop
```

- [ ] **3.1** Basic controls connects successfully
- [ ] **3.2** Emergency stop message appears
- [ ] **3.3** Ctrl+C triggers emergency stop (10 stop commands sent)
- [ ] **3.4** Program exits cleanly

#### Test 2: Camera Feed
```bash
# Should show camera feed in browser
python src/basic_cam.py
# Open browser: http://localhost:8080
```

- [ ] **3.5** Camera feed appears in browser
- [ ] **3.6** FPS counter shows reasonable rate (>5 FPS)
- [ ] **3.7** Ctrl+C exits cleanly

#### Test 3: TTS System
```bash
# Test TTS (if installed)
python3 << EOF
from src.tts_module import TextToSpeech
tts = TextToSpeech(engine='auto')
if tts.available:
    tts.speak("Hello, this is a test")
    print("✓ TTS working")
else:
    print("⚠️ TTS not available")
EOF
```

- [ ] **3.8** TTS speaks test message OR shows "TTS not available"
- [ ] **3.9** No errors or warnings

### Phase 4: Safety Verification

#### Test Emergency Stop
```bash
# Start basic_controls, then test emergency stop:
# 1. Press Ctrl+C - should see "🚨 EMERGENCY STOP" message
# 2. In another terminal: kill -TERM <pid> - should trigger emergency stop
# 3. Close terminal - emergency stop should still trigger
```

- [ ] **4.1** Ctrl+C triggers emergency stop (10 commands sent)
- [ ] **4.2** `kill -TERM` triggers emergency stop
- [ ] **4.3** Closing terminal triggers emergency stop via atexit
- [ ] **4.4** All tests show "✓ Emergency stop commands sent"

### Phase 5: First Motor Test (HIGH RISK - PROCEED WITH CAUTION)

**⚠️ WARNING:** This will activate motors. Be ready to hit emergency stop!

**Setup:**
- [ ] **5.1** Robot is elevated (wheels NOT touching ground) OR on blocks
- [ ] **5.2** Clear 2+ meter radius around robot
- [ ] **5.3** Hand is near power button
- [ ] **5.4** Emergency stop button accessible
- [ ] **5.5** Only ONE person operating, others at safe distance

**Procedure:**
```bash
python src/basic_controls.py eth0

# In the program:
# 1. Type 'mp' - Change to Prepare mode
# 2. Wait for mode change confirmation
# 3. Type 'md' - Change back to Damping mode
# 4. Ctrl+C to exit
```

- [ ] **5.6** Mode change to Prepare successful (robot stands up IF on ground)
- [ ] **5.7** Mode change to Damping successful (robot relaxes)
- [ ] **5.8** Emergency stop (Ctrl+C) immediately stops robot
- [ ] **5.9** NO unexpected movements
- [ ] **5.10** Robot responds predictably

### Phase 6: Recognition System Test (No Motor Control)

```bash
# Run recognition system (does NOT control motors)
python src/smart_recognition.py

# Open browser: http://localhost:8080
```

- [ ] **6.1** Web interface loads
- [ ] **6.2** Camera feed shows
- [ ] **6.3** YOLO detection working (if available)
- [ ] **6.4** Face detection working
- [ ] **6.5** Can interact with system
- [ ] **6.6** Ctrl+C exits cleanly
- [ ] **6.7** Database file created: `smart_database.json`
- [ ] **6.8** Backup file exists: `smart_database.json.backup`

### Phase 7: Jetson Optimization Test (If on Jetson)

```bash
# Test with Jetson optimizations
python src/smart_recognition.py --frame-skip 2 --max-resolution 640
```

- [ ] **7.1** FPS improves with frame-skip
- [ ] **7.2** Detection still works
- [ ] **7.3** No crashes or errors
- [ ] **7.4** CPU/GPU temperature acceptable: `sudo tegrastats`

---

## 🔴 EMERGENCY PROCEDURES

### If Robot Starts Moving Unexpectedly:

1. **Press Ctrl+C immediately** - Emergency stop will trigger
2. **If terminal frozen:** Close terminal window - atexit will trigger
3. **If still moving:** Power button on robot
4. **Last resort:** Physical emergency stop button

### If Program Crashes:

1. Emergency stop will trigger automatically (atexit)
2. Verify robot stopped: Check wheels/motors
3. Check logs for crash cause
4. Fix issue before restarting

### If Network Drops:

1. Robot will continue last command (THIS IS DANGEROUS)
2. Power off robot immediately
3. Fix network before restarting
4. TODO: Add heartbeat check to detect network loss

---

## ⚠️ KNOWN LIMITATIONS

### What's NOT Protected:

1. **Network loss** - Robot continues last command if network drops
   - **Mitigation:** Keep robot in damping mode when testing
   - **TODO:** Add heartbeat monitoring

2. **Power loss** - Robot falls/slumps if power lost
   - **Mitigation:** Ensure stable power supply
   - **Mitigation:** Keep clear space around robot

3. **SIGKILL** - Cannot catch `kill -9`, cleanup won't run
   - **Mitigation:** Never use `kill -9`, use Ctrl+C or `kill -TERM`

4. **Hardware failure** - Software can't prevent hardware issues
   - **Mitigation:** Regular hardware inspection
   - **Mitigation:** Have manual emergency stop ready

---

## 📊 TEST RESULTS LOG

**Date:** _______________
**Tester:** _______________
**K1 Serial:** _______________

| Test | Result | Notes |
|------|--------|-------|
| 1.1-1.8 Environment | ☐ Pass ☐ Fail | |
| 2.1-2.6 Dependencies | ☐ Pass ☐ Fail | |
| 3.1-3.4 Basic Connection | ☐ Pass ☐ Fail | |
| 3.5-3.7 Camera Feed | ☐ Pass ☐ Fail | |
| 3.8-3.9 TTS System | ☐ Pass ☐ Fail | |
| 4.1-4.4 Emergency Stop | ☐ Pass ☐ Fail | |
| 5.6-5.10 First Motor Test | ☐ Pass ☐ Fail | |
| 6.1-6.8 Recognition System | ☐ Pass ☐ Fail | |
| 7.1-7.4 Jetson Optimization | ☐ Pass ☐ Fail | |

**Overall Status:** ☐ Safe to Proceed ☐ Issues Found ☐ Needs Review

**Issues/Notes:**
_______________________________________________________________
_______________________________________________________________
_______________________________________________________________

---

## ✅ READY TO TEST?

**Before you proceed, answer YES to ALL:**

- [ ] I have completed ALL items in Phase 1-4
- [ ] Emergency stop has been tested and works
- [ ] I have a clear plan for emergency situations
- [ ] Robot is in a safe configuration (elevated or clear space)
- [ ] I am ready to hit emergency stop at any moment
- [ ] I have read and understood the KNOWN LIMITATIONS
- [ ] Someone else knows I'm testing (for safety)

**If ANY checkbox is unchecked, DO NOT PROCEED.**

---

## 📞 SUPPORT

- **Issues:** Check `/home/user/booster_k1/README.md`
- **Optimization:** Check `/home/user/booster_k1/JETSON_OPTIMIZATION.md`
- **Roadmap:** Check `/home/user/booster_k1/ROADMAP.md`

---

**Last Updated:** 2025-11-16
**Safety Patches Version:** 1.0
**Critical Fixes Applied:** 4/4

**REMEMBER: Safety first. When in doubt, DON'T.**
