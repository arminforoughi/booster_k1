from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode, B1HandIndex, GripperControlMode, Position, Orientation, Posture, GripperMotionParameter, GetModeResponse, Quaternion, Frame, Transform, DexterousFingerParameter
import sys, time, random
import termios
import tty
import select
import threading
import atexit
import signal

# Try to import voice controller (optional)
try:
    from voice_controller import VoiceController
    VOICE_AVAILABLE = True
except ImportError:
    VOICE_AVAILABLE = False
    print("⚠️  Voice commands not available. Install: pip install SpeechRecognition pyaudio")

def get_error_message(error_code):
    """Convert error code to human-readable message"""
    error_messages = {
        0: "Success",
        100: "Request timeout - Robot not responding. Check network connection and robot power.",
        400: "Bad request - Invalid parameters",
        500: "Internal server error",
        501: "Server refused the request",
        502: "State transition failed - Robot may not be ready for this mode change",
        -1: "Invalid request - Request not sent yet"
    }
    return error_messages.get(error_code, f"Unknown error code: {error_code}")

# Global client reference for emergency stop
_global_client = None

def emergency_stop():
    """
    🚨 EMERGENCY STOP - Send multiple stop commands to ensure robot stops
    Called on any exit scenario (Ctrl+C, kill, crash, etc.)
    """
    global _global_client
    if _global_client is not None:
        print("\n" + "="*60)
        print("🚨 EMERGENCY STOP - Sending stop commands to robot")
        print("="*60)

        # Send stop command 10 times to ensure receipt
        for i in range(10):
            try:
                _global_client.Move(0.0, 0.0, 0.0)
                time.sleep(0.01)
            except Exception as e:
                if i == 0:  # Only print error once
                    print(f"⚠️  Error sending stop command: {e}")

        # Verify robot stopped
        try:
            # Give robot time to process commands
            time.sleep(0.1)
            print("✓ Emergency stop commands sent")
        except:
            pass

def signal_handler(sig, frame):
    """Handle signals (SIGTERM, SIGINT) with emergency stop"""
    print(f"\n🛑 Received signal {sig}")
    emergency_stop()
    sys.exit(0)

# Register emergency stop for all exit scenarios
atexit.register(emergency_stop)
signal.signal(signal.SIGTERM, signal_handler)  # kill command
signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C

def verify_connection(client: B1LocoClient):
    """Verify connection to robot by getting current mode"""
    print("Verifying connection to robot...")
    gm: GetModeResponse = GetModeResponse()
    res = client.GetMode(gm)
    if res == 0:
        mode_names = {
            RobotMode.kDamping: "Damping",
            RobotMode.kPrepare: "Prepare",
            RobotMode.kWalking: "Walking",
            RobotMode.kCustom: "Custom",
            RobotMode.kUnknown: "Unknown"
        }
        mode_name = mode_names.get(gm.mode, f"Unknown mode ({gm.mode})")
        print(f"✓ Connection successful! Current robot mode: {mode_name}")
        return True
    
    print(f"✗ Connection failed: {get_error_message(res)}")
    return False

def get_key():
    """Get a single keypress without waiting for Enter (non-blocking)"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

class DynamicController:
    """Handles dynamic movement controls with continuous command sending"""
    def __init__(self, client: B1LocoClient):
        self.client = client
        self.running = False
        self.active_keys = {}  # key -> last_press_time
        self.lock = threading.Lock()
        self.control_thread = None
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.key_timeout = 0.15  # Release key if not pressed for 150ms
        
    def start(self):
        """Start the control loop"""
        if self.running:
            return
        self.running = True
        # SAFETY: Non-daemon thread ensures cleanup completes before exit
        self.control_thread = threading.Thread(target=self._control_loop, daemon=False)
        self.control_thread.start()

    def stop(self):
        """Stop the control loop safely"""
        print("Stopping control loop...")
        self.running = False

        # Wait for control thread to finish
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
            if self.control_thread.is_alive():
                print("⚠️  WARNING: Control thread did not stop cleanly!")

        # Send multiple stop commands to ensure robot stops
        print("Sending stop commands to robot...")
        for i in range(5):
            try:
                self.client.Move(0.0, 0.0, 0.0)
                time.sleep(0.02)
            except:
                pass

        with self.lock:
            self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.client.Move(0.0, 0.0, 0.0)
        
    def set_key(self, key, active):
        """Set a key as active or inactive"""
        with self.lock:
            if active:
                self.active_keys[key] = time.time()
            else:
                self.active_keys.pop(key, None)
            self._update_velocity()
    
    def _update_velocity(self):
        """Update velocity based on active keys"""
        current_time = time.time()
        # Remove keys that haven't been pressed recently
        keys_to_remove = [k for k, t in self.active_keys.items() 
                         if current_time - t > self.key_timeout]
        for k in keys_to_remove:
            self.active_keys.pop(k, None)
        
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        
        # Forward/backward movement
        if 'w' in self.active_keys:
            self.x = 0.8
        elif 's' in self.active_keys:
            self.x = -0.2
            
        # Left/right strafe
        if 'a' in self.active_keys:
            self.y = 0.2
        elif 'd' in self.active_keys:
            self.y = -0.2
            
        # Rotation (can combine with forward/backward movement)
        if 'q' in self.active_keys:
            self.z = 0.5  # Increased rotation speed for better turning
        elif 'e' in self.active_keys:
            self.z = -0.5  # Increased rotation speed for better turning
    
    def _control_loop(self):
        """Continuously send move commands while running"""
        while self.running:
            with self.lock:
                # Update velocity (removes stale keys)
                self._update_velocity()
                x, y, z = self.x, self.y, self.z
            
            # Always send commands (including stop when x=y=z=0)
            self.client.Move(x, y, z)
            time.sleep(0.05)  # Send commands at ~20Hz

def hand_rock(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 0
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 0
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 0
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 0
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 0
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Rock hand failed: error = {res}")

    time.sleep(0.2)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 0
    finger5_param.force = 200
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Rock hand thumb failed: error = {res}")

def hand_scissor(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 0
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 0
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 1000
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 1000
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 0
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 0
    finger5_param.force = 200
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Scissor hand failed: error = {res}")

def hand_paper(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 1000
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 1000
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 1000
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 1000
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 1000
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 1000
    finger5_param.force = 200
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Paper hand failed: error = {res}")

def hand_grasp(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 350
    finger0_param.force = 400
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 350
    finger1_param.force = 400
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 350
    finger2_param.force = 400
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 350
    finger3_param.force = 400
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 350
    finger4_param.force = 400
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 350
    finger5_param.force = 400
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Grasp hand failed: error = {res}")

def hand_ok(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 1000
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 1000
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 1000
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 500
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 400
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 350
    finger5_param.force = 200
    finger5_param.speed = 1000
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Ok hand failed: error = {res}")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])

    client = B1LocoClient()
    client.Init()

    # Register client for emergency stop
    global _global_client
    _global_client = client
    print("✓ Emergency stop system active")

    # Wait a moment for connection to establish
    print("Initializing connection...")
    time.sleep(1.0)
    
    x, y, z, yaw, pitch = 0.0, 0.0, 0.0, 0.0, 0.0
    res = 0
    hand_action_count = 0

    print("=" * 60)
    print("Booster B1 Locomotion Client - Interactive Mode")
    print("=" * 60)
    
    # Verify connection
    connection_ok = verify_connection(client)
    if not connection_ok:
        print("\n⚠️  WARNING: Could not verify connection to robot.")
        print("   Please check:")
        print("   1. Robot is powered on")
        print("   2. Network interface 'eth0' is correct")
        print("   3. Robot and computer are on the same network")
        print("   4. Robot is ready to accept commands")
        print("\n   You can still try commands, but they may timeout.")
    
    print("\nAvailable commands:")
    print("  Mode: mp (Prepare), md (Damping), mw (Walking), mc (Custom)")
    print("  Move: w/a/s/d/q/e (hold keys for continuous movement, release to stop)")
    print("        w=forward, a=left strafe, s=backward, d=right strafe")
    print("        q=rotate left, e=rotate right (can combine with w/s for turning while moving)")
    print("  Head: hd (down), hu (up), hr (right), hl (left), ho (origin)")
    print("  Hand: hand-down, hand-up, grasp, ok, paper, scissor, rock")
    print("  Other: gm (get mode), gft (get transform), mhel, gopenl")
    print("  Special: Press 'i' to enter interactive mode for other commands")
    if VOICE_AVAILABLE:
        print("  Voice: Say commands like 'forward', 'turn left', 'stop', etc.")
    print("=" * 60)
    print("\nDYNAMIC CONTROLS MODE - Press movement keys directly (no Enter needed)")
    print("Press 'i' for interactive mode, Ctrl+C to exit")
    if VOICE_AVAILABLE:
        print("Press 'v' to toggle voice commands on/off")
    
    # Initialize dynamic controller
    dynamic_controller = DynamicController(client)
    dynamic_controller.start()
    
    # Initialize voice controller (if available)
    voice_controller = None
    
    def handle_voice_command(cmd: str):
        """Handle voice commands by simulating key presses"""
        if cmd in ['w', 'a', 's', 'd', 'q', 'e']:
            # Movement commands - activate key
            dynamic_controller.set_key(cmd, True)
            # Auto-release after a short duration for voice commands
            def release_key():
                time.sleep(1.0)  # Move for 1 second
                dynamic_controller.set_key(cmd, False)
            threading.Thread(target=release_key, daemon=True).start()
        elif cmd == 'stop':
            # Stop all movement
            for k in ['w', 'a', 's', 'd', 'q', 'e']:
                dynamic_controller.set_key(k, False)
        elif cmd == 'mp':
            client.ChangeMode(RobotMode.kPrepare)
        elif cmd == 'md':
            client.ChangeMode(RobotMode.kDamping)
        elif cmd == 'mw':
            client.ChangeMode(RobotMode.kWalking)
        elif cmd == 'mc':
            client.ChangeMode(RobotMode.kCustom)
        elif cmd == 'hd':
            client.RotateHead(1.0, 0.0)
        elif cmd == 'hu':
            client.RotateHead(-0.3, 0.0)
        elif cmd == 'hr':
            client.RotateHead(0.0, -0.785)
        elif cmd == 'hl':
            client.RotateHead(0.0, 0.785)
        elif cmd == 'ho':
            client.RotateHead(0.0, 0.0)
        elif cmd == 'grasp':
            hand_grasp(client)
        elif cmd == 'ok':
            hand_ok(client)
        elif cmd == 'paper':
            hand_paper(client)
        elif cmd == 'scissor':
            hand_scissor(client)
        elif cmd == 'rock':
            hand_rock(client)
    
    if VOICE_AVAILABLE:
        try:
            voice_controller = VoiceController(handle_voice_command)
            print("\n💡 Voice commands available! Press 'v' to enable/disable")
        except Exception as e:
            print(f"⚠️  Could not initialize voice controller: {e}")
            voice_controller = None
    
    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Set terminal to raw mode for single-key input
        tty.setraw(sys.stdin.fileno())
        
        yaw, pitch = 0.0, 0.0
        interactive_mode = False
        
        while True:
            # Check for keyboard input (non-blocking)
            key = get_key()
            
            if key is not None:
                key = key.lower()
                
                # Toggle interactive mode with 'i'
                if key == 'i':
                    interactive_mode = not interactive_mode
                    if interactive_mode:
                        # Restore terminal for normal input
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        print("\n[Interactive Mode] Type commands and press Enter (or 'i' to return to dynamic mode):")
                    else:
                        # Back to raw mode
                        tty.setraw(sys.stdin.fileno())
                        print("\n[Dynamic Mode] Press movement keys directly:")
                    continue
                
                # Toggle voice commands with 'v'
                if key == 'v' and VOICE_AVAILABLE and voice_controller:
                    if voice_controller.is_running:
                        voice_controller.stop()
                        print("\n🔇 Voice commands DISABLED")
                    else:
                        voice_controller.start()
                        print("\n🎤 Voice commands ENABLED")
                    continue
                
                # Handle Ctrl+C
                if ord(key) == 3:  # Ctrl+C
                    break
                
                if interactive_mode:
                    # In interactive mode, restore terminal temporarily for input()
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    if key == '\r' or key == '\n':
                        # User pressed Enter, wait for full command
                        try:
                            input_cmd = input().strip()
                            if input_cmd:
                                res = 0
                                if input_cmd == "mp":
                                    res = client.ChangeMode(RobotMode.kPrepare)
                                elif input_cmd == "md":
                                    res = client.ChangeMode(RobotMode.kDamping)
                                elif input_cmd == "mw":
                                    res = client.ChangeMode(RobotMode.kWalking)
                                elif input_cmd == 'mc':
                                    res = client.ChangeMode(RobotMode.kCustom)
                                elif input_cmd == "stop":
                                    dynamic_controller.set_key('w', False)
                                    dynamic_controller.set_key('a', False)
                                    dynamic_controller.set_key('s', False)
                                    dynamic_controller.set_key('d', False)
                                    dynamic_controller.set_key('q', False)
                                    dynamic_controller.set_key('e', False)
                                elif input_cmd == "hd":
                                    yaw, pitch = 0.0, 1.0
                                    res = client.RotateHead(pitch, yaw)
                                elif input_cmd == "hu":
                                    yaw, pitch = 0.0, -0.3
                                    res = client.RotateHead(pitch, yaw)
                                elif input_cmd == "hr":
                                    yaw, pitch = -0.785, 0.0
                                    res = client.RotateHead(pitch, yaw)
                                elif input_cmd == "hl":
                                    yaw, pitch = 0.785, 0.0
                                    res = client.RotateHead(pitch, yaw)
                                elif input_cmd == "ho":
                                    yaw, pitch = 0.0, 0.0
                                    res = client.RotateHead(pitch, yaw)
                                elif input_cmd == "mhel":
                                    tar_posture = Posture()
                                    tar_posture.position = Position(0.35, 0.25, 0.1)
                                    tar_posture.orientation = Orientation(-1.57, -1.57, 0.0)
                                    res = client.MoveHandEndEffectorV2(tar_posture, 2000, B1HandIndex.kLeftHand)
                                elif input_cmd == "gopenl":
                                    motion_param = GripperMotionParameter()
                                    motion_param.position = 500
                                    motion_param.force = 100
                                    motion_param.speed = 100
                                    res = client.ControlGripper(motion_param, GripperControlMode.kPosition, B1HandIndex.kLeftHand)
                                elif input_cmd == "gft":
                                    src = Frame.kBody
                                    dst = Frame.kLeftHand
                                    transform: Transform = Transform()
                                    res = client.GetFrameTransform(src, dst, transform)
                                    if res == 0:
                                        print(f"Transform: {transform}")
                                elif input_cmd == "gm":
                                    gm: GetModeResponse = GetModeResponse()
                                    res = client.GetMode(gm)
                                    if res == 0:
                                        mode_names = {
                                            RobotMode.kDamping: "Damping",
                                            RobotMode.kPrepare: "Prepare",
                                            RobotMode.kWalking: "Walking",
                                            RobotMode.kCustom: "Custom",
                                            RobotMode.kUnknown: "Unknown"
                                        }
                                        mode_name = mode_names.get(gm.mode, f"Unknown ({gm.mode})")
                                        print(f"Current mode: {mode_name}")
                                elif input_cmd == "hcm-start":
                                    res = client.SwitchHandEndEffectorControlMode(True)
                                elif input_cmd == "hcm-stop":
                                    res = client.SwitchHandEndEffectorControlMode(False)
                                elif input_cmd == "hand-down":
                                    tar_posture = Posture()
                                    tar_posture.position = Position(0.28, -0.25, 0.05)
                                    tar_posture.orientation = Orientation(0.0, 0.0, 0.0)
                                    res = client.MoveHandEndEffector(tar_posture, 1000, B1HandIndex.kRightHand)
                                    time.sleep(0.3)
                                    hand_action_count += 1
                                    r_num = random.randint(0, 2)
                                    if r_num == 0:
                                        hand_rock(client)
                                    elif r_num == 1:
                                        hand_scissor(client)
                                    else:
                                        hand_paper(client)
                                elif input_cmd == "hand-up":
                                    tar_posture = Posture()
                                    tar_posture.position = Position(0.25, -0.3, 0.25)
                                    tar_posture.orientation = Orientation(0.0, -1.0, 0.0)
                                    res = client.MoveHandEndEffector(tar_posture, 1000, B1HandIndex.kRightHand)
                                    time.sleep(0.3)
                                    hand_paper(client)
                                elif input_cmd == "grasp":
                                    hand_grasp(client)
                                elif input_cmd == "ok":
                                    hand_ok(client)
                                elif input_cmd == "paper":
                                    hand_paper(client)
                                elif input_cmd == "scissor":
                                    hand_scissor(client)
                                elif input_cmd == "rock":
                                    hand_rock(client)
                                
                                if res != 0:
                                    error_msg = get_error_message(res)
                                    print(f"Request failed: error = {res} ({error_msg})")
                        except EOFError:
                            pass
                    # Back to raw mode
                    tty.setraw(sys.stdin.fileno())
                else:
                    # Dynamic mode - handle movement keys
                    if key in ['w', 'a', 's', 'd', 'q', 'e']:
                        dynamic_controller.set_key(key, True)
                    elif key == '\x1b':  # ESC key
                        # Release all keys
                        for k in ['w', 'a', 's', 'd', 'q', 'e']:
                            dynamic_controller.set_key(k, False)
                    # Note: Key releases are automatically detected by timeout in DynamicController
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        dynamic_controller.stop()
        if voice_controller:
            voice_controller.stop()
        print("\n\nShutting down...")

if __name__ == "__main__":
    main()
