#!/usr/bin/env python3
"""
Web-based Camera Feed Viewer with YOLO Detection
Supports face detection and general object detection
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

class YOLODetector:
    """YOLO detector wrapper"""

    def __init__(self, model_type='yolov8n', detection_type='face', confidence=0.5):
        self.detection_type = detection_type
        self.confidence = confidence
        self.model = None

        try:
            from ultralytics import YOLO

            if detection_type == 'face':
                # Use YOLOv8 face detection model
                print("Loading YOLOv8 face detection model...")
                # You can download a face-specific model or use general detection
                # For now, we'll use person detection as a proxy
                self.model = YOLO(f'{model_type}.pt')
                self.target_classes = [0]  # Person class
                print("Note: Using person detection. For better face detection, install a face-specific YOLO model")
            else:
                # General object detection
                print(f"Loading {model_type} model...")
                self.model = YOLO(f'{model_type}.pt')
                self.target_classes = None  # All classes

            print("Model loaded successfully!")
            self.available = True

        except ImportError:
            print("ERROR: ultralytics not installed. Install with: pip3 install ultralytics")
            print("Falling back to OpenCV face detection...")
            self.available = False
            self.use_opencv_face = True

            # Load OpenCV Haar Cascade for face detection
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)

        except Exception as e:
            print(f"ERROR loading YOLO: {e}")
            print("Falling back to OpenCV face detection...")
            self.available = False
            self.use_opencv_face = True

            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)

    def detect(self, frame):
        """Run detection on frame and return annotated image"""
        if self.available and self.model is not None:
            # YOLO detection
            results = self.model(frame, conf=self.confidence, verbose=False)

            # Draw results on frame
            annotated_frame = results[0].plot()

            return annotated_frame
        else:
            # OpenCV face detection fallback
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            annotated_frame = frame.copy()
            for (x, y, w, h) in faces:
                cv2.rectangle(annotated_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(annotated_frame, 'Face', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Add face count
            cv2.putText(annotated_frame, f'Faces: {len(faces)}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            return annotated_frame


class CameraSubscriber(Node):
    """ROS2 node that subscribes to camera and runs detection"""

    def __init__(self, detector, show_stereo=False, process_right=False):
        super().__init__('camera_yolo_detector')

        self.bridge = CvBridge()
        self.detector = detector
        self.latest_left_frame = None
        self.latest_right_frame = None
        self.show_stereo = show_stereo
        self.process_right = process_right

        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

        # Use booster camera bridge topics
        left_topic = '/booster_camera_bridge/image_left_raw'
        right_topic = '/booster_camera_bridge/image_right_raw'

        self.get_logger().info(f'Left camera topic: {left_topic}')
        if show_stereo:
            self.get_logger().info(f'Right camera topic: {right_topic}')

        # Subscribe to left camera
        self.left_subscription = self.create_subscription(
            Image,
            left_topic,
            self.left_callback,
            10
        )

        # Subscribe to right camera if requested
        if show_stereo:
            self.right_subscription = self.create_subscription(
                Image,
                right_topic,
                self.right_callback,
                10
            )

    def convert_nv12_to_bgr(self, msg):
        """Convert NV12 ROS image to BGR OpenCV image"""
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

        if elapsed > 1.0:  # Update every second
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time

    def left_callback(self, msg):
        """Callback for left camera image messages"""
        try:
            # Convert to BGR
            cv_image = self.convert_nv12_to_bgr(msg)

            # Run detection
            detected_frame = self.detector.detect(cv_image)

            # Update FPS
            self.update_fps()

            # Add FPS counter
            cv2.putText(detected_frame, f'FPS: {self.fps:.1f}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            self.latest_left_frame = detected_frame

        except Exception as e:
            self.get_logger().error(f'Error processing left image: {str(e)}')

    def right_callback(self, msg):
        """Callback for right camera image messages"""
        try:
            # Convert to BGR
            cv_image = self.convert_nv12_to_bgr(msg)

            if self.process_right:
                # Run detection on right camera too
                detected_frame = self.detector.detect(cv_image)
                self.latest_right_frame = detected_frame
            else:
                # Just show raw feed
                self.latest_right_frame = cv_image

        except Exception as e:
            self.get_logger().error(f'Error processing right image: {str(e)}')


class CameraHTTPHandler(BaseHTTPRequestHandler):
    """HTTP request handler for serving camera images"""

    camera_node = None

    def log_message(self, format, *args):
        """Override to suppress HTTP request logs"""
        pass

    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            # Serve HTML page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>YOLO Detection Feed</title>
                <style>
                    body {
                        font-family: Arial, sans-serif;
                        background-color: #1e1e1e;
                        color: #ffffff;
                        margin: 0;
                        padding: 20px;
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                    }
                    h1 {
                        margin-bottom: 10px;
                    }
                    .info {
                        color: #888;
                        margin-bottom: 20px;
                    }
                    .feed-container {
                        display: flex;
                        gap: 20px;
                        flex-wrap: wrap;
                        justify-content: center;
                    }
                    .feed-box {
                        border: 2px solid #444;
                        padding: 10px;
                        border-radius: 8px;
                        background-color: #2d2d2d;
                    }
                    .feed-box h2 {
                        margin-top: 0;
                        margin-bottom: 10px;
                        font-size: 18px;
                        color: #4CAF50;
                    }
                    img {
                        max-width: 640px;
                        height: auto;
                        display: block;
                        border-radius: 4px;
                    }
                </style>
            </head>
            <body>
                <h1>YOLO Detection Feed</h1>
                <div class="info">Real-time object/face detection</div>
                <div class="feed-container">
                    <div class="feed-box">
                        <h2>Detection Feed</h2>
                        <img id="left-feed" src="/left" alt="Detection Feed">
                    </div>
                    """ + ("""
                    <div class="feed-box">
                        <h2>Right Camera</h2>
                        <img id="right-feed" src="/right" alt="Right Camera">
                    </div>
                    """ if self.camera_node and self.camera_node.show_stereo else "") + """
                </div>
                <script>
                    function refreshImage(id, src) {
                        const img = document.getElementById(id);
                        const newSrc = src + '?t=' + new Date().getTime();
                        img.src = newSrc;
                    }

                    setInterval(() => {
                        refreshImage('left-feed', '/left');
                    }, 100);

                    """ + ("""
                    setInterval(() => {
                        refreshImage('right-feed', '/right');
                    }, 100);
                    """ if self.camera_node and self.camera_node.show_stereo else "") + """
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())

        elif self.path.startswith('/left'):
            if self.camera_node and self.camera_node.latest_left_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()

                _, buffer = cv2.imencode('.jpg', self.camera_node.latest_left_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No feed available')

        elif self.path.startswith('/right'):
            if self.camera_node and self.camera_node.latest_right_frame is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()

                _, buffer = cv2.imencode('.jpg', self.camera_node.latest_right_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.wfile.write(buffer.tobytes())
            else:
                self.send_error(503, 'No feed available')
        else:
            self.send_error(404, 'Not found')


def spin_ros(node):
    """Spin ROS2 node in a separate thread"""
    rclpy.spin(node)


def main():
    parser = argparse.ArgumentParser(description='YOLO detection on camera feed')
    parser.add_argument('--model', type=str, default='yolov8n',
                       choices=['yolov8n', 'yolov8s', 'yolov8m', 'yolov8l', 'yolov8x'],
                       help='YOLO model size (n=nano, s=small, m=medium, l=large, x=xlarge)')
    parser.add_argument('--detection', type=str, default='face',
                       choices=['face', 'object'],
                       help='Detection type: face or general object detection')
    parser.add_argument('--confidence', type=float, default=0.5,
                       help='Confidence threshold (0.0-1.0)')
    parser.add_argument('--stereo', action='store_true',
                       help='Show both cameras')
    parser.add_argument('--process-right', action='store_true',
                       help='Run detection on right camera too (slower)')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Host to bind web server to')
    parser.add_argument('--port', type=int, default=8080,
                       help='Port for web server')

    args = parser.parse_args()

    print(f'\n{"="*60}')
    print(f'YOLO Detection Camera Viewer')
    print(f'{"="*60}')

    # Initialize detector
    detector = YOLODetector(
        model_type=args.model,
        detection_type=args.detection,
        confidence=args.confidence
    )

    # Initialize ROS2
    rclpy.init()

    # Create camera subscriber node
    camera_node = CameraSubscriber(
        detector=detector,
        show_stereo=args.stereo,
        process_right=args.process_right
    )

    # Set the camera node for the HTTP handler
    CameraHTTPHandler.camera_node = camera_node

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(camera_node,), daemon=True)
    ros_thread.start()

    # Start HTTP server
    server_address = (args.host, args.port)
    httpd = HTTPServer(server_address, CameraHTTPHandler)

    print(f'Detection type: {args.detection}')
    print(f'Model: {args.model}')
    print(f'Confidence threshold: {args.confidence}')
    print(f'Web server: http://{args.host}:{args.port}')
    print(f'\nOpen this URL in your browser to view the detection feed')
    print(f'Press Ctrl+C to stop')
    print(f'{"="*60}\n')

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n\nShutting down...')
        httpd.shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
