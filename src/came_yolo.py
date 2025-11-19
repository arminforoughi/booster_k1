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
    """YOLO detector wrapper with 3D depth support"""

    def __init__(self, model_type='yolov8n', detection_type='face', confidence=0.5, use_tensorrt=False):
        self.detection_type = detection_type
        self.confidence = confidence
        self.model = None
        self.detections_3d = []  # Store 3D object positions

        try:
            from ultralytics import YOLO

            # Try TensorRT first for Jetson optimization
            model_path = f'{model_type}.engine' if use_tensorrt else f'{model_type}.pt'

            if detection_type == 'face':
                # Use YOLOv8 face detection model
                print(f"Loading YOLOv8 face detection model ({model_path})...")
                # You can download a face-specific model or use general detection
                # For now, we'll use person detection as a proxy
                try:
                    self.model = YOLO(model_path)
                    print(f"✓ Loaded TensorRT model: {model_path}" if use_tensorrt else f"✓ Loaded PyTorch model: {model_path}")
                except:
                    if use_tensorrt:
                        print(f"⚠️  TensorRT model not found, falling back to PyTorch")
                        self.model = YOLO(f'{model_type}.pt')
                    else:
                        raise
                self.target_classes = [0]  # Person class
                print("Note: Using person detection. For better face detection, install a face-specific YOLO model")
            else:
                # General object detection
                print(f"Loading {model_type} model ({model_path})...")
                try:
                    self.model = YOLO(model_path)
                    print(f"✓ Loaded TensorRT model: {model_path}" if use_tensorrt else f"✓ Loaded PyTorch model: {model_path}")
                except:
                    if use_tensorrt:
                        print(f"⚠️  TensorRT model not found, falling back to PyTorch")
                        self.model = YOLO(f'{model_type}.pt')
                    else:
                        raise
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

    def detect(self, frame, depth_frame=None):
        """Run detection on frame and return annotated image with optional 3D positions"""
        if self.available and self.model is not None:
            # YOLO detection
            results = self.model(frame, conf=self.confidence, verbose=False)

            # Draw results on frame
            annotated_frame = results[0].plot()

            # If depth is available, calculate 3D positions
            if depth_frame is not None:
                self.detections_3d = []
                boxes = results[0].boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    # Get center of bounding box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Get depth at center (in mm for ZED camera)
                    if 0 <= center_y < depth_frame.shape[0] and 0 <= center_x < depth_frame.shape[1]:
                        depth_mm = depth_frame[center_y, center_x]
                        depth_m = depth_mm / 1000.0  # Convert to meters

                        # Store 3D detection
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        self.detections_3d.append({
                            'class_id': class_id,
                            'class_name': self.model.names[class_id] if hasattr(self.model, 'names') else f'class_{class_id}',
                            'confidence': confidence,
                            'bbox': (int(x1), int(y1), int(x2), int(y2)),
                            'center_2d': (center_x, center_y),
                            'depth_m': depth_m,
                            '3d_position': self._pixel_to_3d(center_x, center_y, depth_m)
                        })

                        # Draw depth on frame
                        if depth_m > 0 and depth_m < 20:  # Reasonable depth range
                            cv2.putText(annotated_frame, f'{depth_m:.2f}m',
                                      (int(x1), int(y1) - 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            return annotated_frame

    def _pixel_to_3d(self, u, v, depth):
        """Convert pixel coordinates and depth to 3D position (camera frame)
        Assumes standard pinhole camera model - update with actual camera calibration"""
        # TODO: Get these from /camera/camera_info topic
        fx = 700.0  # Focal length X (placeholder - get from camera_info)
        fy = 700.0  # Focal length Y
        cx = 640.0  # Principal point X (image center)
        cy = 360.0  # Principal point Y

        # Convert to 3D coordinates (meters)
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return (x, y, z)

    def get_detections_3d(self):
        """Return list of 3D detections"""
        return self.detections_3d
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
    """ROS2 node that subscribes to camera and runs detection with depth"""

    def __init__(self, detector, show_stereo=False, process_right=False, use_depth=True):
        super().__init__('camera_yolo_detector')

        self.bridge = CvBridge()
        self.detector = detector
        self.latest_left_frame = None
        self.latest_right_frame = None
        self.latest_depth_frame = None
        self.show_stereo = show_stereo
        self.process_right = process_right
        self.use_depth = use_depth

        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

        # Use booster camera bridge topics
        left_topic = '/booster_camera_bridge/image_left_raw'
        right_topic = '/booster_camera_bridge/image_right_raw'
        depth_topic = '/booster_camera_bridge/depth_raw'

        self.get_logger().info(f'Left camera topic: {left_topic}')
        if show_stereo:
            self.get_logger().info(f'Right camera topic: {right_topic}')
        if use_depth:
            self.get_logger().info(f'Depth topic: {depth_topic}')

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

        # Subscribe to depth if requested
        if use_depth:
            self.depth_subscription = self.create_subscription(
                Image,
                depth_topic,
                self.depth_callback,
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

    def depth_callback(self, msg):
        """Callback for depth image messages"""
        try:
            # Depth is typically mono16 or 32FC1
            if msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            self.latest_depth_frame = depth_image

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def left_callback(self, msg):
        """Callback for left camera image messages"""
        try:
            # Convert to BGR
            cv_image = self.convert_nv12_to_bgr(msg)

            # Run detection with depth if available
            detected_frame = self.detector.detect(cv_image, self.latest_depth_frame)

            # Update FPS
            self.update_fps()

            # Add FPS counter
            cv2.putText(detected_frame, f'FPS: {self.fps:.1f}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Add 3D detection count
            detections_3d = self.detector.get_detections_3d()
            if detections_3d:
                cv2.putText(detected_frame, f'3D Objects: {len(detections_3d)}', (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

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
    parser = argparse.ArgumentParser(description='YOLO detection on camera feed with 3D depth')
    parser.add_argument('--model', type=str, default='yolov8n',
                       choices=['yolov8n', 'yolov8s', 'yolov8m', 'yolov8l', 'yolov8x'],
                       help='YOLO model size (n=nano, s=small, m=medium, l=large, x=xlarge)')
    parser.add_argument('--detection', type=str, default='face',
                       choices=['face', 'object'],
                       help='Detection type: face or general object detection')
    parser.add_argument('--confidence', type=float, default=0.5,
                       help='Confidence threshold (0.0-1.0)')
    parser.add_argument('--tensorrt', action='store_true',
                       help='Use TensorRT model (.engine) for faster inference on Jetson')
    parser.add_argument('--depth', action='store_true', default=True,
                       help='Enable 3D depth integration (default: enabled)')
    parser.add_argument('--no-depth', action='store_false', dest='depth',
                       help='Disable 3D depth integration')
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
    print(f'YOLO Detection Camera Viewer with 3D Depth')
    print(f'{"="*60}')

    # Initialize detector
    detector = YOLODetector(
        model_type=args.model,
        detection_type=args.detection,
        confidence=args.confidence,
        use_tensorrt=args.tensorrt
    )

    # Initialize ROS2
    rclpy.init()

    # Create camera subscriber node
    camera_node = CameraSubscriber(
        detector=detector,
        show_stereo=args.stereo,
        process_right=args.process_right,
        use_depth=args.depth
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
    print(f'Model: {args.model}{"(TensorRT)" if args.tensorrt else "(PyTorch)"}')
    print(f'Confidence threshold: {args.confidence}')
    print(f'3D Depth: {"Enabled" if args.depth else "Disabled"}')
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
