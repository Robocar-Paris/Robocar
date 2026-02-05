"""
Web Dashboard for Robocar Navigation

Flask-based web server that streams OAK-D camera feed with visual overlays
to a PC browser for real-time monitoring of the autonomous navigation.

Features:
- MJPEG video stream of camera feed
- Visual overlays:
  - Green arrow showing GPS target direction
  - Red boxes around obstacles
  - Status text (distance, action, GPS quality)
- Real-time navigation status API
- Responsive web interface

Usage:
    from visualization.web_dashboard import WebDashboard

    dashboard = WebDashboard(camera, navigator)
    dashboard.start(host='0.0.0.0', port=5000)

    # Access from PC browser: http://<jetson-ip>:5000
"""

import cv2
import math
import time
import threading
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

# Try to import Flask
FLASK_AVAILABLE = False
try:
    from flask import Flask, Response, render_template_string, jsonify
    FLASK_AVAILABLE = True
except ImportError:
    print("[Dashboard] Flask not available - install with: pip install flask")


@dataclass
class OverlayConfig:
    """Configuration for visual overlays."""
    # Colors (BGR format)
    gps_arrow_color: Tuple[int, int, int] = (0, 255, 0)        # Green
    obstacle_box_color: Tuple[int, int, int] = (0, 0, 255)      # Red
    warning_box_color: Tuple[int, int, int] = (0, 165, 255)     # Orange
    text_color: Tuple[int, int, int] = (255, 255, 255)          # White
    text_bg_color: Tuple[int, int, int] = (0, 0, 0)             # Black
    avoiding_left_color: Tuple[int, int, int] = (255, 255, 0)   # Cyan
    avoiding_right_color: Tuple[int, int, int] = (255, 0, 255)  # Magenta

    # Sizes
    arrow_length: int = 80
    arrow_thickness: int = 3
    text_scale: float = 0.6
    text_thickness: int = 2
    box_thickness: int = 2

    # Positions
    status_x: int = 10
    status_y_start: int = 30
    status_line_height: int = 25


class OverlayRenderer:
    """
    Renders visual overlays on camera frames.

    Overlays include:
    - GPS target direction arrow
    - Obstacle detection boxes
    - Navigation status text
    """

    def __init__(self, config: Optional[OverlayConfig] = None):
        """
        Initialize overlay renderer.

        Args:
            config: Overlay configuration
        """
        self.config = config or OverlayConfig()

    def render(
        self,
        frame: np.ndarray,
        nav_status=None,
        depth_analysis=None
    ) -> np.ndarray:
        """
        Render all overlays on the frame.

        Args:
            frame: BGR image from camera
            nav_status: NavigationStatus from SmartNavigator
            depth_analysis: DepthAnalysis from OakDCamera

        Returns:
            Frame with overlays drawn
        """
        if frame is None:
            return self._create_no_signal_frame()

        # Make a copy to avoid modifying original
        output = frame.copy()
        h, w = output.shape[:2]

        # Draw overlays
        self._draw_gps_arrow(output, nav_status)
        self._draw_obstacle_indicators(output, depth_analysis)
        self._draw_status_text(output, nav_status, depth_analysis)
        self._draw_action_indicator(output, nav_status)

        return output

    def _draw_gps_arrow(self, frame: np.ndarray, nav_status):
        """Draw arrow pointing to GPS target."""
        if nav_status is None:
            return

        h, w = frame.shape[:2]
        center_x = w // 2
        center_y = h // 2

        # Get heading error (negative = target is to the left)
        heading_error = nav_status.heading_error_deg

        # Convert to screen coordinates
        # 0 degrees = up, positive = clockwise
        angle_rad = math.radians(-heading_error)  # Negate for screen coords

        # Calculate arrow endpoint
        length = self.config.arrow_length
        end_x = int(center_x + length * math.sin(angle_rad))
        end_y = int(center_y - length * math.cos(angle_rad))

        # Draw arrow
        cv2.arrowedLine(
            frame,
            (center_x, center_y),
            (end_x, end_y),
            self.config.gps_arrow_color,
            self.config.arrow_thickness,
            tipLength=0.3
        )

        # Draw target indicator at top
        cv2.circle(frame, (w // 2, 20), 8, self.config.gps_arrow_color, -1)
        cv2.putText(
            frame, "TARGET",
            (w // 2 - 30, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            self.config.gps_arrow_color, 1
        )

    def _draw_obstacle_indicators(self, frame: np.ndarray, depth_analysis):
        """Draw obstacle detection indicators."""
        if depth_analysis is None:
            return

        h, w = frame.shape[:2]

        # Divide frame into thirds (like depth sectors)
        sector_width = w // 3

        # Left sector indicator
        left_color = self._get_distance_color(depth_analysis.left_min_distance)
        if depth_analysis.left_min_distance < 2.0:
            cv2.rectangle(
                frame,
                (0, h - 60),
                (sector_width, h - 10),
                left_color,
                self.config.box_thickness
            )
            cv2.putText(
                frame, f"L:{depth_analysis.left_avg_distance:.1f}m",
                (10, h - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                left_color, 1
            )

        # Center sector indicator (most important)
        center_color = self._get_distance_color(depth_analysis.center_min_distance)
        if depth_analysis.center_min_distance < 2.0:
            cv2.rectangle(
                frame,
                (sector_width, h - 60),
                (2 * sector_width, h - 10),
                center_color,
                self.config.box_thickness + 1  # Thicker for emphasis
            )
            cv2.putText(
                frame, f"C:{depth_analysis.center_avg_distance:.1f}m",
                (sector_width + 10, h - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                center_color, 1
            )

            # Flash warning for critical distance
            if depth_analysis.center_min_distance < 0.5:
                # Draw filled semi-transparent box
                overlay = frame.copy()
                cv2.rectangle(overlay, (sector_width, 0), (2 * sector_width, h), (0, 0, 255), -1)
                cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
                cv2.putText(
                    frame, "OBSTACLE!",
                    (w // 2 - 60, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255), 2
                )

        # Right sector indicator
        right_color = self._get_distance_color(depth_analysis.right_min_distance)
        if depth_analysis.right_min_distance < 2.0:
            cv2.rectangle(
                frame,
                (2 * sector_width, h - 60),
                (w, h - 10),
                right_color,
                self.config.box_thickness
            )
            cv2.putText(
                frame, f"R:{depth_analysis.right_avg_distance:.1f}m",
                (2 * sector_width + 10, h - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                right_color, 1
            )

    def _draw_status_text(self, frame: np.ndarray, nav_status, depth_analysis):
        """Draw navigation status text."""
        h, w = frame.shape[:2]

        lines = []

        if nav_status:
            # Distance to target
            dist = nav_status.distance_to_target_m
            if dist < float('inf'):
                lines.append(f"Distance to Target: {dist:.1f} m")
            else:
                lines.append("Distance to Target: -- m")

            # Current action
            action_text = nav_status.action.value.replace('_', ' ').upper()
            lines.append(f"Action: {action_text}")

            # GPS quality
            lines.append(f"GPS: {nav_status.gps_quality}")

            # Speed/Steering
            lines.append(f"Speed: {nav_status.speed:.2f} | Steer: {nav_status.steering:.2f}")

        if depth_analysis:
            lines.append(f"Front: {depth_analysis.center_min_distance:.1f}m")

        # Draw text with background
        y = self.config.status_y_start
        for line in lines:
            # Calculate text size for background
            (text_w, text_h), _ = cv2.getTextSize(
                line, cv2.FONT_HERSHEY_SIMPLEX,
                self.config.text_scale, self.config.text_thickness
            )

            # Draw background rectangle
            cv2.rectangle(
                frame,
                (self.config.status_x - 2, y - text_h - 2),
                (self.config.status_x + text_w + 2, y + 4),
                self.config.text_bg_color,
                -1
            )

            # Draw text
            cv2.putText(
                frame, line,
                (self.config.status_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.config.text_scale,
                self.config.text_color,
                self.config.text_thickness
            )

            y += self.config.status_line_height

    def _draw_action_indicator(self, frame: np.ndarray, nav_status):
        """Draw visual indicator for current action."""
        if nav_status is None:
            return

        h, w = frame.shape[:2]
        action = nav_status.action.value

        if action == "avoiding_left":
            # Big left arrow
            pts = np.array([
                [50, h // 2],
                [100, h // 2 - 40],
                [100, h // 2 + 40]
            ], np.int32)
            cv2.fillPoly(frame, [pts], self.config.avoiding_left_color)
            cv2.putText(
                frame, "AVOIDING LEFT",
                (10, h // 2 + 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                self.config.avoiding_left_color, 2
            )

        elif action == "avoiding_right":
            # Big right arrow
            pts = np.array([
                [w - 50, h // 2],
                [w - 100, h // 2 - 40],
                [w - 100, h // 2 + 40]
            ], np.int32)
            cv2.fillPoly(frame, [pts], self.config.avoiding_right_color)
            cv2.putText(
                frame, "AVOIDING RIGHT",
                (w - 180, h // 2 + 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                self.config.avoiding_right_color, 2
            )

        elif action == "arrived":
            # Checkmark
            cv2.putText(
                frame, "ARRIVED!",
                (w // 2 - 80, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                (0, 255, 0), 3
            )

        elif action == "stopping":
            cv2.putText(
                frame, "STOPPED",
                (w // 2 - 60, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (0, 0, 255), 2
            )

    def _get_distance_color(self, distance: float) -> Tuple[int, int, int]:
        """Get color based on distance (green=safe, yellow=warning, red=danger)."""
        if distance < 0.5:
            return (0, 0, 255)      # Red - danger
        elif distance < 1.0:
            return (0, 165, 255)    # Orange - warning
        elif distance < 1.5:
            return (0, 255, 255)    # Yellow - caution
        else:
            return (0, 255, 0)      # Green - safe

    def _create_no_signal_frame(self) -> np.ndarray:
        """Create a 'no signal' frame when camera unavailable."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            frame, "NO CAMERA SIGNAL",
            (180, 240),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            (0, 0, 255), 2
        )
        return frame


# HTML template for web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robocar Navigation Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: #1a1a2e;
            color: #eee;
            min-height: 100vh;
        }
        .header {
            background: #16213e;
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #0f3460;
        }
        .header h1 {
            color: #00ff88;
            font-size: 24px;
        }
        .status-indicator {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            animation: pulse 1.5s infinite;
        }
        .status-dot.connected { background: #00ff88; }
        .status-dot.disconnected { background: #ff4444; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        .container {
            display: flex;
            padding: 20px;
            gap: 20px;
            max-width: 1400px;
            margin: 0 auto;
        }
        .video-panel {
            flex: 2;
        }
        .video-container {
            background: #0f0f0f;
            border-radius: 10px;
            overflow: hidden;
            border: 2px solid #0f3460;
        }
        .video-container img {
            width: 100%;
            display: block;
        }
        .info-panel {
            flex: 1;
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .card {
            background: #16213e;
            border-radius: 10px;
            padding: 15px;
            border: 1px solid #0f3460;
        }
        .card h3 {
            color: #00ff88;
            margin-bottom: 10px;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .metric {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #0f3460;
        }
        .metric:last-child { border-bottom: none; }
        .metric-label { color: #888; }
        .metric-value { font-weight: bold; color: #fff; }
        .metric-value.good { color: #00ff88; }
        .metric-value.warning { color: #ffaa00; }
        .metric-value.danger { color: #ff4444; }
        .action-badge {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-weight: bold;
            text-transform: uppercase;
            font-size: 12px;
        }
        .action-navigating { background: #00ff88; color: #000; }
        .action-avoiding_left { background: #ffff00; color: #000; }
        .action-avoiding_right { background: #ff00ff; color: #fff; }
        .action-arrived { background: #00ff88; color: #000; }
        .action-stopping { background: #ff4444; color: #fff; }
        .action-error { background: #ff0000; color: #fff; }
        .depth-bars {
            display: flex;
            gap: 10px;
            margin-top: 10px;
        }
        .depth-bar {
            flex: 1;
            text-align: center;
        }
        .depth-bar-fill {
            height: 100px;
            background: #0f3460;
            border-radius: 5px;
            position: relative;
            overflow: hidden;
        }
        .depth-bar-level {
            position: absolute;
            bottom: 0;
            left: 0;
            right: 0;
            background: linear-gradient(to top, #00ff88, #ffaa00, #ff4444);
            transition: height 0.3s;
        }
        .depth-bar-label {
            margin-top: 5px;
            font-size: 12px;
            color: #888;
        }
        .depth-bar-value {
            font-weight: bold;
            margin-top: 2px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>ROBOCAR Navigation Dashboard</h1>
        <div class="status-indicator">
            <div class="status-dot connected" id="status-dot"></div>
            <span id="status-text">Connected</span>
        </div>
    </div>

    <div class="container">
        <div class="video-panel">
            <div class="video-container">
                <img src="/video_feed" alt="Camera Feed" id="video-feed">
            </div>
        </div>

        <div class="info-panel">
            <div class="card">
                <h3>Navigation Status</h3>
                <div class="metric">
                    <span class="metric-label">Action</span>
                    <span class="action-badge action-navigating" id="action">NAVIGATING</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Distance to Target</span>
                    <span class="metric-value" id="distance">-- m</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Bearing</span>
                    <span class="metric-value" id="bearing">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Speed</span>
                    <span class="metric-value" id="speed">0.00</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Steering</span>
                    <span class="metric-value" id="steering">0.00</span>
                </div>
            </div>

            <div class="card">
                <h3>GPS Status</h3>
                <div class="metric">
                    <span class="metric-label">Quality</span>
                    <span class="metric-value" id="gps-quality">--</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Position</span>
                    <span class="metric-value" id="gps-position">--, --</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Target</span>
                    <span class="metric-value" id="gps-target">--, --</span>
                </div>
            </div>

            <div class="card">
                <h3>Depth Sensors</h3>
                <div class="depth-bars">
                    <div class="depth-bar">
                        <div class="depth-bar-fill">
                            <div class="depth-bar-level" id="depth-left" style="height: 0%"></div>
                        </div>
                        <div class="depth-bar-label">LEFT</div>
                        <div class="depth-bar-value" id="depth-left-val">--</div>
                    </div>
                    <div class="depth-bar">
                        <div class="depth-bar-fill">
                            <div class="depth-bar-level" id="depth-center" style="height: 0%"></div>
                        </div>
                        <div class="depth-bar-label">CENTER</div>
                        <div class="depth-bar-value" id="depth-center-val">--</div>
                    </div>
                    <div class="depth-bar">
                        <div class="depth-bar-fill">
                            <div class="depth-bar-level" id="depth-right" style="height: 0%"></div>
                        </div>
                        <div class="depth-bar-label">RIGHT</div>
                        <div class="depth-bar-value" id="depth-right-val">--</div>
                    </div>
                </div>
            </div>

            <div class="card">
                <h3>System</h3>
                <div class="metric">
                    <span class="metric-label">Elapsed Time</span>
                    <span class="metric-value" id="elapsed">0.0s</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Frame Rate</span>
                    <span class="metric-value" id="fps">-- fps</span>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Update status data every 200ms
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Update action badge
                    const actionEl = document.getElementById('action');
                    actionEl.textContent = data.action.toUpperCase().replace('_', ' ');
                    actionEl.className = 'action-badge action-' + data.action;

                    // Update metrics
                    document.getElementById('distance').textContent =
                        data.distance < 9999 ? data.distance.toFixed(1) + ' m' : '-- m';
                    document.getElementById('bearing').textContent = data.bearing.toFixed(0) + '°';
                    document.getElementById('speed').textContent = data.speed.toFixed(2);
                    document.getElementById('steering').textContent = data.steering.toFixed(2);

                    // GPS
                    document.getElementById('gps-quality').textContent = data.gps_quality;
                    document.getElementById('gps-position').textContent =
                        data.current_lat.toFixed(6) + ', ' + data.current_lon.toFixed(6);
                    document.getElementById('gps-target').textContent =
                        data.target_lat.toFixed(6) + ', ' + data.target_lon.toFixed(6);

                    // Depth bars
                    const maxDist = 3.0;
                    const leftPct = Math.min(100, (data.depth_left / maxDist) * 100);
                    const centerPct = Math.min(100, (data.depth_center / maxDist) * 100);
                    const rightPct = Math.min(100, (data.depth_right / maxDist) * 100);

                    document.getElementById('depth-left').style.height = leftPct + '%';
                    document.getElementById('depth-center').style.height = centerPct + '%';
                    document.getElementById('depth-right').style.height = rightPct + '%';

                    document.getElementById('depth-left-val').textContent = data.depth_left.toFixed(1) + 'm';
                    document.getElementById('depth-center-val').textContent = data.depth_center.toFixed(1) + 'm';
                    document.getElementById('depth-right-val').textContent = data.depth_right.toFixed(1) + 'm';

                    // System
                    document.getElementById('elapsed').textContent = data.elapsed.toFixed(1) + 's';
                    document.getElementById('fps').textContent = data.fps.toFixed(1) + ' fps';

                    // Connection status
                    document.getElementById('status-dot').className = 'status-dot connected';
                    document.getElementById('status-text').textContent = 'Connected';
                })
                .catch(err => {
                    document.getElementById('status-dot').className = 'status-dot disconnected';
                    document.getElementById('status-text').textContent = 'Disconnected';
                });
        }

        setInterval(updateStatus, 200);
        updateStatus();
    </script>
</body>
</html>
"""


class WebDashboard:
    """
    Flask-based web dashboard for Robocar navigation.

    Provides:
    - MJPEG video stream with overlays
    - Real-time navigation status API
    - Responsive web interface
    """

    def __init__(
        self,
        camera,       # OakDCamera instance
        navigator=None,  # SmartNavigator instance (optional)
        overlay_config: Optional[OverlayConfig] = None
    ):
        """
        Initialize web dashboard.

        Args:
            camera: OAK-D camera for video feed
            navigator: Smart navigator for status (optional)
            overlay_config: Overlay rendering configuration
        """
        self.camera = camera
        self.navigator = navigator
        self.renderer = OverlayRenderer(overlay_config)

        # Flask app
        self._app: Optional[Flask] = None
        self._server_thread: Optional[threading.Thread] = None
        self._running = False

        # Frame statistics
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = time.time()

    def start(self, host: str = '0.0.0.0', port: int = 5000, debug: bool = False):
        """
        Start the web dashboard server.

        Args:
            host: Host address (0.0.0.0 for all interfaces)
            port: Port number
            debug: Enable Flask debug mode
        """
        if not FLASK_AVAILABLE:
            print("[Dashboard] Flask not available")
            return False

        self._app = Flask(__name__)
        self._running = True

        # Register routes
        self._register_routes()

        # Start server in background thread
        self._server_thread = threading.Thread(
            target=self._run_server,
            args=(host, port, debug),
            daemon=True,
            name="WebDashboard"
        )
        self._server_thread.start()

        print(f"[Dashboard] Started at http://{host}:{port}")
        return True

    def stop(self):
        """Stop the web dashboard server."""
        self._running = False
        print("[Dashboard] Stopped")

    def _register_routes(self):
        """Register Flask routes."""

        @self._app.route('/')
        def index():
            """Serve the dashboard HTML."""
            return render_template_string(HTML_TEMPLATE)

        @self._app.route('/video_feed')
        def video_feed():
            """MJPEG video stream endpoint."""
            return Response(
                self._generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @self._app.route('/api/status')
        def api_status():
            """JSON API for navigation status."""
            return jsonify(self._get_status_dict())

    def _run_server(self, host: str, port: int, debug: bool):
        """Run Flask server (in thread)."""
        # Disable Flask logging for cleaner output
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        self._app.run(
            host=host,
            port=port,
            debug=debug,
            threaded=True,
            use_reloader=False
        )

    def _generate_frames(self):
        """Generate MJPEG frames with overlays."""
        while self._running:
            # Get frame and depth
            frame = self.camera.get_frame()
            depth_analysis = self.camera.get_depth_analysis()

            # Get navigation status
            nav_status = None
            if self.navigator:
                nav_status = self.navigator.get_status()

            # Render overlays
            output = self.renderer.render(frame, nav_status, depth_analysis)

            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', output, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ret:
                continue

            frame_bytes = buffer.tobytes()

            # Update FPS counter
            self._frame_count += 1
            now = time.time()
            if now - self._last_fps_time >= 1.0:
                self._fps = self._frame_count / (now - self._last_fps_time)
                self._frame_count = 0
                self._last_fps_time = now

            # Yield MJPEG frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

            # Limit frame rate to ~30 fps
            time.sleep(0.033)

    def _get_status_dict(self) -> dict:
        """Get status as dictionary for JSON API."""
        status = {
            'action': 'stopped',
            'distance': float('inf'),
            'bearing': 0.0,
            'speed': 0.0,
            'steering': 0.0,
            'gps_quality': 'NO_FIX',
            'current_lat': 0.0,
            'current_lon': 0.0,
            'target_lat': 0.0,
            'target_lon': 0.0,
            'depth_left': 0.0,
            'depth_center': 0.0,
            'depth_right': 0.0,
            'elapsed': 0.0,
            'fps': self._fps
        }

        if self.navigator:
            nav_status = self.navigator.get_status()
            status.update({
                'action': nav_status.action.value,
                'distance': nav_status.distance_to_target_m,
                'bearing': nav_status.bearing_to_target_deg,
                'speed': nav_status.speed,
                'steering': nav_status.steering,
                'gps_quality': nav_status.gps_quality,
                'current_lat': nav_status.current_lat,
                'current_lon': nav_status.current_lon,
                'target_lat': nav_status.target_lat,
                'target_lon': nav_status.target_lon,
                'depth_left': nav_status.left_sector_distance_m,
                'depth_center': nav_status.obstacle_distance_m,
                'depth_right': nav_status.right_sector_distance_m,
                'elapsed': nav_status.elapsed_time_s
            })

        depth_analysis = self.camera.get_depth_analysis()
        if depth_analysis:
            status['depth_left'] = depth_analysis.left_avg_distance
            status['depth_center'] = depth_analysis.center_avg_distance
            status['depth_right'] = depth_analysis.right_avg_distance

        return status

    @property
    def fps(self) -> float:
        """Get current frame rate."""
        return self._fps


# Test code
if __name__ == '__main__':
    print("[Dashboard] Testing web dashboard...")

    # Mock camera for testing
    class MockCamera:
        def get_frame(self):
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "TEST FRAME", (200, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
            return frame

        def get_depth_analysis(self):
            from driver.oak_d_camera import DepthAnalysis
            return DepthAnalysis(
                left_avg_distance=2.5,
                right_avg_distance=1.8,
                center_avg_distance=1.2,
                left_min_distance=2.0,
                right_min_distance=1.5,
                center_min_distance=1.0,
                recommended_direction='left',
                obstacle_detected=True,
                timestamp=time.time()
            )

    camera = MockCamera()
    dashboard = WebDashboard(camera)

    if dashboard.start(port=5000):
        print("[Dashboard] Running at http://localhost:5000")
        print("[Dashboard] Press Ctrl+C to stop")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Dashboard] Stopping...")
            dashboard.stop()
