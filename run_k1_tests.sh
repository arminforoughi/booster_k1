#!/bin/bash

# K-1 Robot Test Script
# Runs the safety checklist tests directly on the robot

ROBOT_IP="192.168.88.153"
ROBOT_USER="booster"
ROBOT_PASSWORD="123456"
ROBOT_PATH="/home/booster/booster_k1"

echo "============================================================"
echo "🤖 K-1 ROBOT TEST SUITE"
echo "============================================================"
echo "Robot IP: $ROBOT_IP"
echo "Starting tests..."
echo ""

# Install sshpass if needed
if ! command -v sshpass &> /dev/null; then
    echo "Installing sshpass..."
    if [[ "$OSTYPE" == "darwin"* ]]; then
        brew install hudochenkov/sshpass/sshpass
    else
        sudo apt-get install -y sshpass
    fi
fi

# Create SSH function with password
ssh_robot() {
    sshpass -p "$ROBOT_PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_IP" "$@"
}

echo "============================================================"
echo "📍 PHASE 1: Testing Connection"
echo "============================================================"

# Test ping
echo "Testing network connectivity..."
if ping -c 3 $ROBOT_IP > /dev/null 2>&1; then
    echo "✅ Robot is reachable at $ROBOT_IP"
else
    echo "❌ Cannot ping robot at $ROBOT_IP"
    exit 1
fi

# Test SSH
echo "Testing SSH connection..."
if ssh_robot "echo 'SSH connection successful'"; then
    echo "✅ SSH connection successful"
else
    echo "❌ SSH connection failed"
    exit 1
fi

echo ""
echo "============================================================"
echo "📍 PHASE 2: Checking Dependencies"
echo "============================================================"

# Check Python
echo "Checking Python version..."
ssh_robot "python3 --version"

# Check ROS2
echo "Checking ROS2..."
ssh_robot "source /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null && ros2 --version 2>/dev/null || echo 'ROS2 not found'"

# Check camera topics
echo "Checking camera topics..."
ssh_robot "source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null | grep camera || source /opt/ros/foxy/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null | grep camera || echo 'No camera topics found'"

# Check TTS
echo "Checking TTS..."
ssh_robot "which piper || which espeak || echo 'No TTS installed'"

echo ""
echo "============================================================"
echo "📍 PHASE 3: Testing Basic Controls (Robot in DAMPING mode)"
echo "============================================================"
echo "⚠️  IMPORTANT: Ensure robot is in DAMPING mode (motors off)"
read -p "Press Enter when robot is in DAMPING mode..."

echo "Testing basic_controls.py connection..."
ssh_robot "cd $ROBOT_PATH && timeout 5 python3 src/basic_controls.py 2>&1 | head -30"

echo ""
echo "============================================================"
echo "📍 PHASE 4: Testing Camera Feed"
echo "============================================================"

echo "Starting camera feed test..."
ssh_robot "cd $ROBOT_PATH && timeout 3 python3 src/basic_cam.py 2>&1 | grep -E 'Camera topic|Web server|Started' | head -10" &
CAMERA_PID=$!
sleep 3
kill $CAMERA_PID 2>/dev/null
echo "✅ Camera feed test complete"

echo ""
echo "============================================================"
echo "📍 PHASE 5: Testing TTS System"
echo "============================================================"

ssh_robot "cd $ROBOT_PATH && python3 -c \"
from src.tts_module import TextToSpeech
tts = TextToSpeech(engine='auto')
if tts.available:
    print('✅ TTS available using:', tts.engine)
    tts.speak('K1 robot test successful', blocking=True)
    print('TTS test complete')
else:
    print('⚠️ TTS not available')
\""

echo ""
echo "============================================================"
echo "📍 PHASE 6: Running Recognition System Test"
echo "============================================================"

echo "Starting recognition system (5 second test)..."
ssh_robot "cd $ROBOT_PATH && timeout 5 python3 src/smart_recognition.py --no-voice 2>&1 | grep -E 'Smart Recognition|Web interface|Enabled features' | head -10" &
REC_PID=$!
sleep 5
kill $REC_PID 2>/dev/null
echo "✅ Recognition system test complete"

echo ""
echo "============================================================"
echo "✅ AUTOMATED TESTS COMPLETE"
echo "============================================================"
echo ""
echo "📋 NEXT STEPS FOR MANUAL TESTING:"
echo ""
echo "1. For MOTOR TEST (HIGH RISK):"
echo "   - Ensure robot is elevated OR has 2+ meter clear space"
echo "   - Have emergency stop button ready"
echo "   - SSH into robot: sshpass -p '$ROBOT_PASSWORD' ssh $ROBOT_USER@$ROBOT_IP"
echo "   - cd $ROBOT_PATH"
echo "   - python3 src/basic_controls.py"
echo "   - Type 'mp' to enter Prepare mode (robot will stand)"
echo "   - Type 'md' to return to Damping mode"
echo "   - Use WASD keys for movement (if in Walking mode)"
echo ""
echo "2. To view camera feed:"
echo "   Open browser: http://$ROBOT_IP:8080"
echo ""
echo "3. To test recognition system:"
echo "   SSH and run: python3 src/smart_recognition.py"
echo "   Open browser: http://$ROBOT_IP:8080"
echo ""
echo "============================================================"