#!/usr/bin/env python3
"""
Smart Recognition System for Booster K1
Combines YOLO object detection + Face recognition + TTS
Asks for names when unknown people/objects are detected
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import argparse
import time
import json
import os
from datetime import datetime

# Import our modules
from face_recognition import FaceRecognizer
from tts_module import TextToSpeech
from voice_listener import VoiceListener

class SmartRecognizer:
    """Unified recognizer for objects and people"""

    def __init__(self, database_path='smart_database.json', use_yolo=True, use_deepface=True, use_tts=True, use_voice=True):
        self.database_path = database_path
        self.database = self._load_database()

        # Initialize YOLO
        self.yolo_available = False
        self.yolo_model = None
        if use_yolo:
            try:
                from ultralytics import YOLO
                print("Loading YOLOv8 model...")
                self.yolo_model = YOLO('yolov8n.pt')
                self.yolo_available = True
                print("✓ YOLO loaded successfully")
            except Exception as e:
                print(f"⚠️  YOLO not available: {e}")

        # Initialize face recognizer
        self.face_recognizer = FaceRecognizer(use_deepface=use_deepface)

        # Initialize TTS with auto-detection (tries Piper -> espeak -> pyttsx3)
        self.tts = None
        if use_tts:
            self.tts = TextToSpeech(engine='auto')

        # Initialize Voice Listener
        self.voice_listener = None
        if use_voice:
            try:
                self.voice_listener = VoiceListener()
                if self.voice_listener.available:
                    print("✓ Voice listener initialized")
                else:
                    self.voice_listener = None
            except Exception as e:
                print(f"⚠️  Voice listener not available: {e}")
                self.voice_listener = None

        # Track what we've asked about recently (to avoid spam)
        self.recently_asked = {}  # {name: timestamp}
        self.ask_cooldown = 30.0  # seconds between asking about same thing

        # Track pending voice learning
        self.waiting_for_name = False
        self.pending_face_img = None

    def _load_database(self):
        """Load object database"""
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
            'people': {},      # {name: {first_seen, last_seen, times_seen, conversations: []}}
            'objects': {},     # {name: {class, first_seen, last_seen, times_seen}}
            'opt_out': [],     # List of people who didn't want to connect
            'version': '1.0'
        }

    def _save_database(self):
        """Save database to file"""
        try:
            with open(self.database_path, 'w') as f:
                json.dump(self.database, f, indent=2)
        except Exception as e:
            print(f"✗ Error saving database: {e}")

    def detect_and_recognize(self, frame):
        """
        Main detection and recognition pipeline
        Returns annotated frame and list of detections
        """
        detections = []

        # First, detect faces for people
        faces = self.face_recognizer.detect_faces(frame)

        for (x, y, w, h) in faces:
            face_img = frame[y:y+h, x:x+w]

            # Try to recognize the person
            if self.face_recognizer.deepface_available:
                name, confidence = self.face_recognizer.recognize_face(face_img)

                if name is None or confidence < 0.6:
                    # Unknown person!
                    name = "Unknown Person"
                    self._handle_unknown_person(face_img, (x, y, w, h))
                else:
                    # Known person, update database and greet
                    self._update_person(name)

                    # Greet if we haven't greeted them recently
                    greet_key = f"greeted_{name}"
                    if greet_key not in self.recently_asked:
                        self.greet_known_person(name)
                        # Don't greet again for a while
                        self.recently_asked[greet_key] = time.time()

                detections.append({
                    'type': 'person',
                    'name': name,
                    'confidence': confidence,
                    'bbox': (x, y, w, h)
                })

                # Draw on frame
                color = (0, 255, 0) if name != "Unknown Person" else (0, 165, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                label = f"{name}"
                if confidence > 0:
                    label += f" ({confidence:.2f})"
                cv2.putText(frame, label, (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            else:
                # No face recognition, just show as detected person
                detections.append({
                    'type': 'person',
                    'name': 'Person (no recognition)',
                    'confidence': 0.0,
                    'bbox': (x, y, w, h)
                })
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, 'Person', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Run YOLO for general object detection
        if self.yolo_available:
            results = self.yolo_model(frame, conf=0.5, verbose=False)

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Get class and confidence
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = result.names[cls]

                    # Skip if it's a person (we already handled faces)
                    if class_name == 'person':
                        continue

                    # Check if this object has a custom name
                    custom_name = self._get_object_name(class_name, (x1, y1, x2, y2))

                    if custom_name:
                        display_name = custom_name
                        self._update_object(custom_name, class_name)
                    else:
                        display_name = class_name
                        # Ask what this is if we don't have a custom name
                        self._handle_unknown_object(class_name, (x1, y1, x2, y2))

                    detections.append({
                        'type': 'object',
                        'name': display_name,
                        'class': class_name,
                        'confidence': conf,
                        'bbox': (x1, y1, x2, y2)
                    })

                    # Draw on frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    label = f"{display_name} ({conf:.2f})"
                    cv2.putText(frame, label, (x1, y1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        return frame, detections

    def _handle_unknown_person(self, face_img, bbox):
        """Handle detection of unknown person"""
        key = "unknown_person"

        # Check if we recently asked about this
        if key in self.recently_asked:
            elapsed = time.time() - self.recently_asked[key]
            if elapsed < self.ask_cooldown:
                return

        # Don't ask if we're already waiting for a name
        if self.waiting_for_name:
            return

        # Ask via TTS and listen for response
        if self.tts and self.tts.available and self.voice_listener and self.voice_listener.available:
            # Mark that we're waiting
            self.waiting_for_name = True
            self.pending_face_img = face_img.copy()
            self.recently_asked[key] = time.time()

            # Ask and listen in a separate thread to not block camera processing
            def ask_and_learn():
                # Natural greeting - ask if they want to connect
                response = self.voice_listener.ask_and_listen(
                    self.tts,
                    "Hello! I haven't met you yet. Would you like to connect?",
                    timeout=8,
                    phrase_time_limit=5
                )

                if response:
                    response_lower = response.lower().strip()

                    # Check for negative responses
                    decline_phrases = ['no', 'nope', 'not now', "don't", 'later', 'pass']

                    if any(phrase in response_lower for phrase in decline_phrases):
                        # User declined to connect
                        print("User declined to connect")
                        self.tts.speak("No problem! Have a great day.", blocking=False)

                        # Add to opt-out list
                        if 'opt_out' not in self.database:
                            self.database['opt_out'] = []
                        self.database['opt_out'].append({
                            'timestamp': datetime.now().isoformat(),
                            'note': 'Declined to connect'
                        })
                        self._save_database()
                    else:
                        # User wants to connect - have a natural conversation
                        print("User wants to connect - starting conversation")
                        conversation = {
                            'timestamp': datetime.now().isoformat(),
                            'exchanges': []
                        }

                        # Start conversation - ask for name
                        name_response = self.voice_listener.ask_and_listen(
                            self.tts,
                            "Great! What's your name?",
                            timeout=8,
                            phrase_time_limit=5
                        )

                        if name_response:
                            name = name_response.strip().title()
                            conversation['exchanges'].append({
                                'k1': "What's your name?",
                                'person': name_response
                            })

                            print(f"Met new person: {name}")

                            # Natural follow-up
                            self.tts.speak(f"Nice to meet you, {name}!", blocking=True)

                            # Ask something natural to keep conversation going
                            interest_response = self.voice_listener.ask_and_listen(
                                self.tts,
                                "What brings you here today?",
                                timeout=10,
                                phrase_time_limit=8
                            )

                            if interest_response:
                                conversation['exchanges'].append({
                                    'k1': "What brings you here today?",
                                    'person': interest_response
                                })

                                # Acknowledge their response
                                self.tts.speak("That's great! I'll remember that.", blocking=True)

                            # Save the person with conversation
                            if self.pending_face_img is not None:
                                # Create person entry with conversation
                                person_data = {
                                    'name': name,
                                    'first_seen': datetime.now().isoformat(),
                                    'last_seen': datetime.now().isoformat(),
                                    'times_seen': 1,
                                    'conversations': [conversation]
                                }

                                # Add to face recognizer
                                success = self.face_recognizer.add_person(name, self.pending_face_img)

                                if success:
                                    # Update database with person info
                                    if name not in self.database['people']:
                                        self.database['people'][name] = person_data
                                    else:
                                        # Add conversation to existing person
                                        self.database['people'][name]['conversations'].append(conversation)
                                        self.database['people'][name]['last_seen'] = datetime.now().isoformat()
                                        self.database['people'][name]['times_seen'] += 1

                                    self._save_database()
                                    print(f"✓ Connected with {name}")

                                    # Friendly goodbye
                                    self.tts.speak(f"Great to meet you, {name}! I'll recognize you next time.", blocking=False)
                                else:
                                    print(f"✗ Failed to save face for {name}")
                        else:
                            # Didn't get name
                            self.tts.speak("I didn't catch that. Feel free to say hello anytime!", blocking=False)
                else:
                    print("No response heard, will try again later")

                # Reset waiting state
                self.waiting_for_name = False
                self.pending_face_img = None

            # Start in background thread
            threading.Thread(target=ask_and_learn, daemon=True).start()

        elif self.tts and self.tts.available:
            # Fallback: just ask (use web interface to teach)
            self.tts.speak("I see someone I don't recognize. Who is this?")
            self.recently_asked[key] = time.time()

    def _handle_unknown_object(self, class_name, bbox):
        """Handle detection of object without custom name"""
        key = f"unknown_object_{class_name}"

        # Check if we recently asked about this
        if key in self.recently_asked:
            elapsed = time.time() - self.recently_asked[key]
            if elapsed < self.ask_cooldown:
                return

        # Ask via TTS (but less frequently than people)
        if self.tts and self.tts.available and class_name not in ['chair', 'couch', 'table']:
            # Don't ask about very common objects
            self.tts.speak(f"I see a {class_name}. Does it have a name?")
            self.recently_asked[key] = time.time()

    def _get_object_name(self, class_name, bbox):
        """Get custom name for an object if it exists"""
        # For now, just check if we have this object class with a custom name
        if class_name in self.database['objects']:
            return self.database['objects'][class_name].get('custom_name')
        return None

    def _update_person(self, name):
        """Update person in database"""
        now = datetime.now().isoformat()
        if name not in self.database['people']:
            self.database['people'][name] = {
                'first_seen': now,
                'last_seen': now,
                'times_seen': 1,
                'conversations': []  # Always include conversations for consistency
            }
        else:
            self.database['people'][name]['last_seen'] = now
            self.database['people'][name]['times_seen'] += 1
            # Ensure conversations field exists for backward compatibility
            if 'conversations' not in self.database['people'][name]:
                self.database['people'][name]['conversations'] = []
        self._save_database()

    def _update_object(self, custom_name, class_name):
        """Update object in database"""
        now = datetime.now().isoformat()
        if custom_name not in self.database['objects']:
            self.database['objects'][custom_name] = {
                'class': class_name,
                'first_seen': now,
                'last_seen': now,
                'times_seen': 1
            }
        else:
            self.database['objects'][custom_name]['last_seen'] = now
            self.database['objects'][custom_name]['times_seen'] += 1
        self._save_database()

    def learn_person(self, name, face_img):
        """Teach the system a person's name"""
        if self.face_recognizer.add_person(name, face_img):
            self._update_person(name)
            return True
        return False

    def greet_known_person(self, name):
        """Greet a person K1 already knows with context from past conversations"""
        if not self.tts or not self.tts.available:
            return

        # Get person info from database
        person_info = self.database.get('people', {}).get(name)

        if person_info:
            times_seen = person_info.get('times_seen', 1)
            has_conversations = 'conversations' in person_info and len(person_info.get('conversations', [])) > 0

            # Greet with appropriate familiarity
            if times_seen == 1:
                greeting = f"Hello again, {name}!"
            elif times_seen < 5:
                greeting = f"Hey {name}, good to see you!"
            else:
                greeting = f"Welcome back, {name}!"

            self.tts.speak(greeting, blocking=False)
        else:
            # Fallback if person not in database (shouldn't happen)
            self.tts.speak(f"Hello, {name}!", blocking=False)

    def name_object(self, class_name, custom_name):
        """Give an object a custom name"""
        if class_name not in self.database['objects']:
            self.database['objects'][class_name] = {
                'class': class_name,
                'custom_name': custom_name,
                'first_seen': datetime.now().isoformat(),
                'last_seen': datetime.now().isoformat(),
                'times_seen': 0
            }
        else:
            self.database['objects'][class_name]['custom_name'] = custom_name

        self._save_database()

        if self.tts and self.tts.available:
            self.tts.speak(f"Okay, I'll remember this {class_name} as {custom_name}.")

        return True


class SmartRecognitionNode(Node):
    """ROS2 node for smart recognition"""

    def __init__(self, recognizer: SmartRecognizer):
        super().__init__('smart_recognition_node')

        self.bridge = CvBridge()
        self.recognizer = recognizer
        self.latest_frame = None
        self.latest_detections = []

        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

        # Learning mode
        self.learning_mode = False
        self.learning_name = None

        # Subscribe to camera
        left_topic = '/booster_camera_bridge/image_left_raw'
        self.get_logger().info(f'Camera topic: {left_topic}')

        self.subscription = self.create_subscription(
            Image,
            left_topic,
            self.camera_callback,
            10
        )

    def convert_nv12_to_bgr(self, msg):
        """Convert NV12 ROS image to BGR"""
        if msg.encoding == 'nv12':
            height = msg.height
            width = msg.width
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            yuv = img_data.reshape((int(height * 1.5), width))
            cv_image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            return cv_image
        else:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def update_fps(self):
        """Update FPS counter"""
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_fps_time

        if elapsed > 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time

    def camera_callback(self, msg):
        """Process camera frames"""
        try:
            frame = self.convert_nv12_to_bgr(msg)

            # Learning mode: capture face for person
            if self.learning_mode and self.learning_name:
                faces = self.recognizer.face_recognizer.detect_faces(frame)
                if len(faces) > 0:
                    x, y, w, h = faces[0]
                    face_img = frame[y:y+h, x:x+w]
                    self.recognizer.learn_person(self.learning_name, face_img)
                    self.learning_mode = False
                    self.learning_name = None

            # Run recognition
            annotated_frame, detections = self.recognizer.detect_and_recognize(frame)
            self.latest_detections = detections

            # Update FPS
            self.update_fps()

            # Add stats to frame
            cv2.putText(annotated_frame, f'FPS: {self.fps:.1f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(annotated_frame, f'Detections: {len(detections)}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            people_count = len(self.recognizer.database['people'])
            objects_count = len(self.recognizer.database['objects'])
            cv2.putText(annotated_frame, f'Known: {people_count}P / {objects_count}O', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if self.learning_mode:
                cv2.putText(annotated_frame, f'LEARNING: {self.learning_name}', (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self.latest_frame = annotated_frame

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def start_learning(self, name):
        """Start learning a person's name"""
        self.learning_mode = True
        self.learning_name = name


class SmartRecognitionHTTPHandler(BaseHTTPRequestHandler):
    """HTTP handler for web interface"""

    camera_node = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self._serve_main_page()
        elif self.path.startswith('/feed'):
            self._serve_feed()
        elif self.path.startswith('/learn'):
            self._handle_learn()
        elif self.path.startswith('/name_object'):
            self._handle_name_object()
        elif self.path.startswith('/stats'):
            self._serve_stats()
        else:
            self.send_error(404)

    def _serve_main_page(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Smart Recognition System</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    background-color: #1e1e1e;
                    color: #ffffff;
                    margin: 0;
                    padding: 20px;
                }
                h1 { color: #4CAF50; }
                .container {
                    max-width: 1200px;
                    margin: 0 auto;
                }
                .controls {
                    background-color: #2d2d2d;
                    padding: 15px;
                    border-radius: 8px;
                    margin-bottom: 20px;
                }
                input, button {
                    padding: 8px;
                    margin: 5px;
                    border-radius: 4px;
                }
                button {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    cursor: pointer;
                }
                button:hover { background-color: #45a049; }
                img {
                    max-width: 100%;
                    border-radius: 8px;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>🤖 Smart Recognition System</h1>

                <div class="controls">
                    <h3>Learn Person</h3>
                    <input type="text" id="personName" placeholder="Enter person's name">
                    <button onclick="learnPerson()">Learn Person</button>
                </div>

                <div class="controls">
                    <h3>Name Object</h3>
                    <input type="text" id="objectClass" placeholder="Object class (e.g., bottle)">
                    <input type="text" id="objectName" placeholder="Custom name">
                    <button onclick="nameObject()">Name Object</button>
                </div>

                <img id="feed" src="/feed" style="width: 100%; margin-top: 20px;">

                <div id="stats" style="margin-top: 20px;"></div>
            </div>

            <script>
                function refreshFeed() {
                    document.getElementById('feed').src = '/feed?t=' + new Date().getTime();
                }
                setInterval(refreshFeed, 100);

                function learnPerson() {
                    const name = document.getElementById('personName').value;
                    if (name) {
                        fetch('/learn?name=' + encodeURIComponent(name))
                            .then(r => r.text())
                            .then(alert);
                        document.getElementById('personName').value = '';
                    }
                }

                function nameObject() {
                    const cls = document.getElementById('objectClass').value;
                    const name = document.getElementById('objectName').value;
                    if (cls && name) {
                        fetch('/name_object?class=' + encodeURIComponent(cls) + '&name=' + encodeURIComponent(name))
                            .then(r => r.text())
                            .then(alert);
                    }
                }

                function refreshStats() {
                    fetch('/stats')
                        .then(r => r.json())
                        .then(data => {
                            let html = '<h3>Database</h3>';
                            html += '<p>People: ' + Object.keys(data.people).length + '</p>';
                            html += '<p>Objects: ' + Object.keys(data.objects).length + '</p>';
                            document.getElementById('stats').innerHTML = html;
                        });
                }
                setInterval(refreshStats, 5000);
                refreshStats();
            </script>
        </body>
        </html>
        """
        self.wfile.write(html.encode())

    def _serve_feed(self):
        if self.camera_node and self.camera_node.latest_frame is not None:
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            _, buffer = cv2.imencode('.jpg', self.camera_node.latest_frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.wfile.write(buffer.tobytes())
        else:
            self.send_error(503)

    def _handle_learn(self):
        import urllib.parse
        query = urllib.parse.urlparse(self.path).query
        params = urllib.parse.parse_qs(query)
        name = params.get('name', [''])[0]

        if name and self.camera_node:
            self.camera_node.start_learning(name)
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"Learning {name}...".encode())
        else:
            self.send_error(400)

    def _handle_name_object(self):
        import urllib.parse
        query = urllib.parse.urlparse(self.path).query
        params = urllib.parse.parse_qs(query)
        class_name = params.get('class', [''])[0]
        custom_name = params.get('name', [''])[0]

        if class_name and custom_name and self.camera_node:
            self.camera_node.recognizer.name_object(class_name, custom_name)
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"Named {class_name} as {custom_name}".encode())
        else:
            self.send_error(400)

    def _serve_stats(self):
        if self.camera_node:
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            stats = self.camera_node.recognizer.database
            self.wfile.write(json.dumps(stats).encode())
        else:
            self.send_error(503)


def spin_ros(node):
    rclpy.spin(node)


def ask_permission_to_start(recognizer):
    """
    Soft auto-start: Ask permission before starting full system
    Returns True if user wants to start, False otherwise
    """
    if not recognizer.tts or not recognizer.tts.available:
        # No TTS, just auto-start
        return True

    if not recognizer.voice_listener or not recognizer.voice_listener.available:
        # No voice input, just auto-start
        return True

    print("\n🤖 Asking permission to start...")

    # Ask for permission
    response = recognizer.voice_listener.ask_and_listen(
        recognizer.tts,
        "Hello! I am ready. Would you like me to start the recognition system?",
        timeout=10,
        phrase_time_limit=5
    )

    if response:
        response_lower = response.lower().strip()
        print(f"Response: {response}")

        # Check for affirmative responses
        affirmative = ['yes', 'yeah', 'yep', 'sure', 'okay', 'ok', 'start', 'go', 'begin', 'please']
        negative = ['no', 'nope', 'not now', 'later', 'wait', 'stop']

        if any(word in response_lower for word in affirmative):
            recognizer.tts.speak("Great! Starting now.", blocking=True)
            return True
        elif any(word in response_lower for word in negative):
            recognizer.tts.speak("Okay, I'll wait. Say start when you're ready.", blocking=True)
            return False
        else:
            # Unclear response, ask for confirmation
            recognizer.tts.speak("I didn't quite understand. Starting anyway.", blocking=True)
            return True
    else:
        # No response, auto-start after a moment
        print("No response detected, auto-starting...")
        recognizer.tts.speak("No response detected. Starting recognition system.", blocking=True)
        return True


def standby_mode(recognizer):
    """
    Standby mode - wait for user to say 'start' or similar
    """
    print("\n💤 Standby mode - say 'start' when ready")
    print("Press Ctrl+C to exit")

    if not recognizer.voice_listener or not recognizer.voice_listener.available:
        print("⚠️  Voice listener not available, exiting standby")
        return True

    # Listen continuously for start command
    start_commands = ['start', 'begin', 'go', 'ready', 'let\'s go']

    recognizer.voice_listener.start_continuous_listening()

    # Simple callback to check for start commands
    start_detected = threading.Event()

    def check_for_start(text):
        text_lower = text.lower()
        if any(cmd in text_lower for cmd in start_commands):
            print(f"✓ Start command detected: {text}")
            recognizer.tts.speak("Starting recognition system now!", blocking=False)
            start_detected.set()

    # Set callback
    original_callback = recognizer.voice_listener.callback
    recognizer.voice_listener.callback = check_for_start

    try:
        # Wait for start command
        while not start_detected.is_set():
            time.sleep(0.5)

        # Restore original callback
        recognizer.voice_listener.callback = original_callback
        recognizer.voice_listener.stop_continuous_listening()

        return True

    except KeyboardInterrupt:
        print("\n\n👋 Exiting standby mode...")
        recognizer.voice_listener.callback = original_callback
        recognizer.voice_listener.stop_continuous_listening()
        return False


def main():
    parser = argparse.ArgumentParser(description='Smart recognition system with voice learning')
    parser.add_argument('--no-yolo', action='store_true', help='Disable YOLO')
    parser.add_argument('--no-deepface', action='store_true', help='Disable DeepFace')
    parser.add_argument('--no-tts', action='store_true', help='Disable TTS')
    parser.add_argument('--no-voice', action='store_true', help='Disable voice learning')
    parser.add_argument('--auto-ask', action='store_true', help='Ask permission before starting (soft auto-start)')
    parser.add_argument('--database', type=str, default='smart_database.json')
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)

    args = parser.parse_args()

    print(f'\n{"="*60}')
    print(f'Smart Recognition System with Voice Learning')
    print(f'{"="*60}')

    # Initialize recognizer (lightweight initialization)
    recognizer = SmartRecognizer(
        database_path=args.database,
        use_yolo=not args.no_yolo,
        use_deepface=not args.no_deepface,
        use_tts=not args.no_tts,
        use_voice=not args.no_voice
    )

    # Soft auto-start: Ask permission if --auto-ask is enabled
    if args.auto_ask:
        should_start = ask_permission_to_start(recognizer)

        if not should_start:
            # Enter standby mode and wait for start command
            should_start = standby_mode(recognizer)

            if not should_start:
                # User cancelled from standby
                print("\n👋 Goodbye!")
                if recognizer.tts:
                    recognizer.tts.speak("Goodbye!", blocking=True)
                    recognizer.tts.stop()
                return

    # Initialize ROS2
    rclpy.init()
    camera_node = SmartRecognitionNode(recognizer=recognizer)
    SmartRecognitionHTTPHandler.camera_node = camera_node

    # Start ROS2
    ros_thread = threading.Thread(target=spin_ros, args=(camera_node,), daemon=True)
    ros_thread.start()

    # Start HTTP server
    httpd = HTTPServer((args.host, args.port), SmartRecognitionHTTPHandler)

    print(f'Web interface: http://{args.host}:{args.port}')
    print(f'Database: {args.database}')

    # Show which features are enabled
    features = []
    if recognizer.yolo_available:
        features.append("YOLO")
    if recognizer.face_recognizer.deepface_available:
        features.append("DeepFace")
    if recognizer.tts and recognizer.tts.available:
        features.append("TTS")
    if recognizer.voice_listener and recognizer.voice_listener.available:
        features.append("Voice Learning")

    print(f'Enabled features: {", ".join(features) if features else "None"}')

    if recognizer.voice_listener and recognizer.voice_listener.available:
        print(f'\n💡 Voice learning enabled! K1 will ask "Who is this?" and listen for your answer.')

    if args.auto_ask:
        print(f'\n🤖 Soft auto-start enabled')

    print(f'\nPress Ctrl+C to stop')
    print(f'{"="*60}\n')

    # Announce startup
    if recognizer.tts and recognizer.tts.available:
        recognizer.tts.speak("Recognition system is now running.")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n\nShutting down...')
        httpd.shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()
        if recognizer.tts:
            recognizer.tts.stop()


if __name__ == '__main__':
    main()
