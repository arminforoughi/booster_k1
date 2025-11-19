#!/usr/bin/env python3
"""
Wave Detection Greeter Bot for Booster K1
Detects hand waves, remembers people, and responds with personalized greetings
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import os
from datetime import datetime
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

# Try to import MediaPipe for hand/pose detection
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("⚠️  MediaPipe not available. Install with: pip install mediapipe")
    print("   Wave detection will use simplified motion detection")

# Import our modules
try:
    from face_recognition import FaceRecognizer
    from tts_module import TextToSpeech
except ImportError:
    print("⚠️  Warning: Could not import local modules")


class WaveDetector:
    """Detects hand waving gestures using pose/hand landmarks"""

    def __init__(self, use_mediapipe=True):
        self.use_mediapipe = use_mediapipe and MEDIAPIPE_AVAILABLE

        if self.use_mediapipe:
            # Initialize MediaPipe Hands
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.mp_draw = mp.solutions.drawing_utils
            print("✓ MediaPipe hand tracking initialized")
        else:
            # Fallback: Simple motion detection
            self.prev_frame_gray = None
            print("✓ Simple motion detection initialized")

        # Wave detection state
        self.hand_positions = deque(maxlen=15)  # Track last 15 hand positions
        self.wave_detected_time = 0
        self.wave_cooldown = 3.0  # seconds between wave detections

    def detect_wave_mediapipe(self, frame):
        """Detect wave gesture using MediaPipe hand tracking"""
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        wave_detected = False
        hand_bbox = None

        if results.multi_hand_landmarks:
            for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

                # Get wrist position (landmark 0)
                wrist = hand_landmarks.landmark[0]
                h, w, _ = frame.shape
                wrist_x = int(wrist.x * w)
                wrist_y = int(wrist.y * h)

                # Track hand position over time
                self.hand_positions.append((wrist_x, wrist_y, time.time()))

                # Analyze wave pattern
                if len(self.hand_positions) >= 10:
                    wave_detected = self._analyze_wave_pattern()

                    if wave_detected:
                        # Calculate bounding box around hand
                        x_coords = [int(lm.x * w) for lm in hand_landmarks.landmark]
                        y_coords = [int(lm.y * h) for lm in hand_landmarks.landmark]
                        hand_bbox = (
                            min(x_coords),
                            min(y_coords),
                            max(x_coords) - min(x_coords),
                            max(y_coords) - min(y_coords)
                        )

                        # Draw wave indicator
                        cv2.putText(frame, "WAVE DETECTED!", (50, 50),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        return wave_detected, hand_bbox, frame

    def detect_wave_simple(self, frame):
        """Fallback: Simple motion-based wave detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        wave_detected = False
        hand_bbox = None

        if self.prev_frame_gray is not None:
            # Compute frame difference
            frame_diff = cv2.absdiff(self.prev_frame_gray, gray)
            thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)

            # Find contours
            contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)

            # Look for significant motion in upper half of frame
            h, w = frame.shape[:2]
            for contour in contours:
                if cv2.contourArea(contour) < 500:
                    continue

                (x, y, w_box, h_box) = cv2.boundingRect(contour)

                # Only consider motion in upper half (where hands usually are)
                if y < h * 0.7:
                    hand_bbox = (x, y, w_box, h_box)
                    cv2.rectangle(frame, (x, y), (x + w_box, y + h_box),
                                (0, 255, 0), 2)

                    # Simple wave heuristic: horizontal motion
                    if w_box > h_box * 1.2:  # Wider than tall
                        wave_detected = True
                        cv2.putText(frame, "WAVE DETECTED!", (50, 50),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        self.prev_frame_gray = gray
        return wave_detected, hand_bbox, frame

    def detect_wave(self, frame):
        """Main wave detection method"""
        # Cooldown check
        if time.time() - self.wave_detected_time < self.wave_cooldown:
            return False, None, frame

        if self.use_mediapipe:
            wave_detected, hand_bbox, frame = self.detect_wave_mediapipe(frame)
        else:
            wave_detected, hand_bbox, frame = self.detect_wave_simple(frame)

        if wave_detected:
            self.wave_detected_time = time.time()

        return wave_detected, hand_bbox, frame

    def _analyze_wave_pattern(self):
        """Analyze hand positions to detect waving motion"""
        if len(self.hand_positions) < 10:
            return False

        # Get x positions and times
        positions = list(self.hand_positions)
        x_positions = [p[0] for p in positions]
        times = [p[2] for p in positions]

        # Check if positions change significantly (side-to-side motion)
        x_range = max(x_positions) - min(x_positions)
        time_span = times[-1] - times[0]

        # Wave pattern: at least 50px horizontal movement over 0.5-2 seconds
        if x_range > 50 and 0.5 < time_span < 2.0:
            # Count direction changes (oscillation)
            direction_changes = 0
            for i in range(1, len(x_positions) - 1):
                if (x_positions[i] > x_positions[i-1] and x_positions[i] > x_positions[i+1]) or \
                   (x_positions[i] < x_positions[i-1] and x_positions[i] < x_positions[i+1]):
                    direction_changes += 1

            # Wave should have at least 2 direction changes (back and forth)
            return direction_changes >= 2

        return False


class WaveGreeterBot:
    """Greeter bot that remembers people and responds to waves"""

    def __init__(self, database_path='wave_greeter_db.json'):
        self.database_path = database_path
        self.database = self._load_database()

        # Initialize wave detector
        self.wave_detector = WaveDetector(use_mediapipe=MEDIAPIPE_AVAILABLE)

        # Initialize face recognizer
        self.face_recognizer = FaceRecognizer(use_deepface=True)

        # Initialize TTS
        self.tts = TextToSpeech(engine='auto')

        # Track recent greetings to avoid spam
        self.recent_greetings = {}  # {name: timestamp}
        self.greeting_cooldown = 30.0  # Don't greet same person for 30 seconds

        print("✓ Wave Greeter Bot initialized")

    def _load_database(self):
        """Load greeter database"""
        if os.path.exists(self.database_path):
            try:
                with open(self.database_path, 'r') as f:
                    return json.load(f)
            except:
                return self._create_empty_database()
        return self._create_empty_database()

    def _create_empty_database(self):
        """Create empty database structure"""
        return {
            'people': {},  # {name: {first_wave, last_wave, wave_count, greetings: []}}
            'version': '1.0'
        }

    def _save_database(self):
        """Save database atomically"""
        temp_path = f"{self.database_path}.tmp"
        try:
            with open(temp_path, 'w') as f:
                json.dump(self.database, f, indent=2)
            os.replace(temp_path, self.database_path)
        except Exception as e:
            print(f"Error saving database: {e}")

    def process_frame(self, frame):
        """Process frame: detect wave, recognize person, greet"""
        # Detect wave
        wave_detected, hand_bbox, annotated_frame = self.wave_detector.detect_wave(frame)

        if wave_detected:
            # Someone waved! Try to recognize them
            person_name = self._recognize_person(frame)

            if person_name:
                self._handle_wave(person_name, frame)
            else:
                # Unknown person waving
                self._handle_unknown_wave(frame)

        return annotated_frame

    def _recognize_person(self, frame):
        """Try to recognize the person in frame"""
        faces = self.face_recognizer.detect_faces(frame)

        if len(faces) > 0:
            # Use the largest face (closest person)
            largest_face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = largest_face
            face_img = frame[y:y+h, x:x+w]

            if self.face_recognizer.deepface_available:
                name, confidence = self.face_recognizer.recognize_face(face_img)
                if name and confidence > 0.6:
                    return name

        return None

    def _handle_wave(self, name, frame):
        """Handle wave from known person"""
        current_time = time.time()

        # Check cooldown
        if name in self.recent_greetings:
            if current_time - self.recent_greetings[name] < self.greeting_cooldown:
                return  # Don't greet again so soon

        # Update database
        if name not in self.database['people']:
            self.database['people'][name] = {
                'first_wave': datetime.now().isoformat(),
                'last_wave': datetime.now().isoformat(),
                'wave_count': 0,
                'greetings': []
            }

        person = self.database['people'][name]
        person['last_wave'] = datetime.now().isoformat()
        person['wave_count'] = person.get('wave_count', 0) + 1

        # Generate personalized greeting
        wave_count = person['wave_count']
        greeting = self._generate_greeting(name, wave_count)

        person['greetings'].append({
            'time': datetime.now().isoformat(),
            'greeting': greeting
        })

        self._save_database()

        # Speak greeting
        if self.tts and self.tts.available:
            self.tts.speak(greeting, blocking=False)

        # Update cooldown
        self.recent_greetings[name] = current_time

        print(f"👋 {name} waved! Greeting: {greeting}")

    def _handle_unknown_wave(self, frame):
        """Handle wave from unknown person"""
        current_time = time.time()

        # Check cooldown for unknown people
        if 'unknown' in self.recent_greetings:
            if current_time - self.recent_greetings['unknown'] < self.greeting_cooldown:
                return

        greeting = "Hello! I don't think we've met yet. I'm K1!"

        if self.tts and self.tts.available:
            self.tts.speak(greeting, blocking=False)

        self.recent_greetings['unknown'] = current_time
        print(f"👋 Unknown person waved! Greeting: {greeting}")

    def _generate_greeting(self, name, wave_count):
        """Generate personalized greeting based on history"""
        if wave_count == 1:
            greetings = [
                f"Hello {name}! Great to meet you!",
                f"Hi {name}! Nice to see you wave!",
                f"Hey {name}! Welcome!"
            ]
        elif wave_count < 5:
            greetings = [
                f"Hey {name}! Good to see you again!",
                f"Hi {name}! How are you today?",
                f"Hello {name}! Welcome back!"
            ]
        elif wave_count < 10:
            greetings = [
                f"Hey {name}! Always nice to see a familiar face!",
                f"Hi {name}! You're becoming a regular!",
                f"Welcome back {name}! Great to see you!"
            ]
        else:
            greetings = [
                f"Hey {name}! You're one of my favorite people!",
                f"Hi {name}! Always a pleasure!",
                f"Welcome {name}! So good to see you again!"
            ]

        import random
        return random.choice(greetings)


class WaveGreeterNode(Node):
    """ROS2 node for wave greeter bot"""

    def __init__(self, greeter: WaveGreeterBot):
        super().__init__('wave_greeter_node')

        self.bridge = CvBridge()
        self.greeter = greeter
        self.latest_frame = None

        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

        # Subscribe to camera
        left_topic = '/booster_camera_bridge/image_left_raw'
        self.get_logger().info(f'Camera topic: {left_topic}')

        self.subscription = self.create_subscription(
            Image,
            left_topic,
            self.camera_callback,
            10
        )

    def camera_callback(self, msg):
        """Process camera frames"""
        try:
            # Convert ROS image to OpenCV
            if msg.encoding == 'nv12':
                height = msg.height
                width = msg.width
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                yuv = img_data.reshape((int(height * 1.5), width))
                frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process frame with greeter
            annotated_frame = self.greeter.process_frame(frame)

            # Update FPS
            self.frame_count += 1
            current_time = time.time()
            elapsed = current_time - self.last_fps_time

            if elapsed > 1.0:
                self.fps = self.frame_count / elapsed
                self.frame_count = 0
                self.last_fps_time = current_time

            # Add FPS counter
            cv2.putText(annotated_frame, f'FPS: {self.fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            self.latest_frame = annotated_frame

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')


# HTTP Server for web viewing
class WaveGreeterHTTPHandler(BaseHTTPRequestHandler):
    """HTTP handler for wave greeter web interface"""

    greeter_node = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>K1 Wave Greeter</title>
                <style>
                    body {
                        font-family: Arial, sans-serif;
                        background: #1e1e1e;
                        color: #fff;
                        text-align: center;
                        padding: 20px;
                    }
                    h1 { color: #00ff00; }
                    img { max-width: 90%; border: 2px solid #00ff00; }
                </style>
            </head>
            <body>
                <h1>👋 K1 Wave Greeter Bot</h1>
                <p>Wave at the camera and I'll greet you!</p>
                <img id="feed" src="/feed" alt="Camera Feed">
                <script>
                    setInterval(() => {
                        document.getElementById('feed').src = '/feed?t=' + new Date().getTime();
                    }, 100);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())

        elif self.path.startswith('/feed'):
            if self.greeter_node and self.greeter_node.latest_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()

                _, buffer = cv2.imencode('.jpg', self.greeter_node.latest_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No feed available')


def main():
    import argparse

    parser = argparse.ArgumentParser(description='K1 Wave Greeter Bot')
    parser.add_argument('--port', type=int, default=8080,
                       help='Web server port')
    parser.add_argument('--no-mediapipe', action='store_true',
                       help='Disable MediaPipe (use simple detection)')

    args = parser.parse_args()

    print(f'\n{"="*60}')
    print(f'K1 Wave Greeter Bot')
    print(f'{"="*60}')
    print(f'Wave detection: {"MediaPipe" if MEDIAPIPE_AVAILABLE and not args.no_mediapipe else "Simple"}')
    print(f'Web interface: http://0.0.0.0:{args.port}')
    print(f'\nWave at the camera to be greeted!')
    print(f'{"="*60}\n')

    # Initialize ROS2
    rclpy.init()

    # Create greeter bot
    greeter = WaveGreeterBot()

    # Create ROS2 node
    greeter_node = WaveGreeterNode(greeter)

    # Set up HTTP handler
    WaveGreeterHTTPHandler.greeter_node = greeter_node

    # Start ROS2 spinning in thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(greeter_node), daemon=True)
    ros_thread.start()

    # Start HTTP server
    httpd = HTTPServer(('0.0.0.0', args.port), WaveGreeterHTTPHandler)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n\nShutting down...')
        httpd.shutdown()
        greeter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
