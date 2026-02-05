"""
OAK-D Lite Camera Driver with Depth Sensing

Driver for Luxonis OAK-D Lite camera using DepthAI.
Provides:
- RGB camera stream
- Stereo depth map
- Depth sector analysis for obstacle avoidance

Usage:
    camera = OakDCamera()
    camera.start()

    frame, depth = camera.get_frame_and_depth()
    left_dist, right_dist = camera.analyze_depth_sectors()

    camera.stop()
"""

import time
import threading
import numpy as np
from typing import Optional, Tuple, Callable
from dataclasses import dataclass

# Try to import DepthAI
DEPTHAI_AVAILABLE = False
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
    print("[OAK-D] DepthAI library loaded")
except ImportError:
    print("[OAK-D] DepthAI not available - camera will be simulated")


@dataclass
class DepthAnalysis:
    """Result of depth sector analysis."""
    left_avg_distance: float      # Average distance in left sector (meters)
    right_avg_distance: float     # Average distance in right sector (meters)
    center_avg_distance: float    # Average distance in center sector (meters)
    left_min_distance: float      # Minimum distance in left sector
    right_min_distance: float     # Minimum distance in right sector
    center_min_distance: float    # Minimum distance in center
    recommended_direction: str    # 'left', 'right', or 'straight'
    obstacle_detected: bool       # True if obstacle in warning zone
    timestamp: float


@dataclass
class CameraConfig:
    """OAK-D camera configuration."""
    rgb_resolution: str = "1080p"     # 1080p, 4K, 720p
    depth_resolution: str = "400p"    # 400p, 480p, 720p, 800p
    fps: int = 30
    # Depth sensing parameters
    depth_min_mm: int = 200           # 20cm minimum depth
    depth_max_mm: int = 10000         # 10m maximum depth
    # Sector analysis
    warning_distance_m: float = 1.5   # Distance to trigger avoidance
    critical_distance_m: float = 0.5  # Distance considered critical


class OakDCamera:
    """
    OAK-D Lite camera driver for Robocar.

    Provides RGB stream and depth analysis for navigation.
    """

    def __init__(self, config: Optional[CameraConfig] = None):
        """
        Initialize OAK-D camera driver.

        Args:
            config: Camera configuration (optional)
        """
        self.config = config or CameraConfig()

        # DepthAI pipeline and device
        self._pipeline: Optional['dai.Pipeline'] = None
        self._device: Optional['dai.Device'] = None

        # Output queues
        self._rgb_queue = None
        self._depth_queue = None

        # Threading
        self._running = False
        self._capture_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Latest frames
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_analysis: Optional[DepthAnalysis] = None
        self._frame_count = 0

        # Callbacks
        self._frame_callback: Optional[Callable] = None

        # Frame dimensions (set after start)
        self.rgb_width = 1920
        self.rgb_height = 1080
        self.depth_width = 640
        self.depth_height = 400

    def start(self) -> bool:
        """
        Start the camera.

        Returns:
            True if successfully started
        """
        if not DEPTHAI_AVAILABLE:
            print("[OAK-D] Running in simulation mode (no DepthAI)")
            self._running = True
            self._capture_thread = threading.Thread(
                target=self._simulate_capture_loop,
                daemon=True,
                name="OakDSimulated"
            )
            self._capture_thread.start()
            return True

        try:
            self._pipeline = self._create_pipeline()
            self._device = dai.Device(self._pipeline)

            # Get output queues
            self._rgb_queue = self._device.getOutputQueue(
                name="rgb",
                maxSize=4,
                blocking=False
            )
            self._depth_queue = self._device.getOutputQueue(
                name="depth",
                maxSize=4,
                blocking=False
            )

            self._running = True
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                daemon=True,
                name="OakDCapture"
            )
            self._capture_thread.start()

            print(f"[OAK-D] Started - RGB: {self.rgb_width}x{self.rgb_height}, "
                  f"Depth: {self.depth_width}x{self.depth_height}")
            return True

        except Exception as e:
            print(f"[OAK-D] Failed to start: {e}")
            return False

    def stop(self):
        """Stop the camera."""
        self._running = False

        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)

        if self._device:
            self._device.close()
            self._device = None

        print("[OAK-D] Stopped")

    def _create_pipeline(self) -> 'dai.Pipeline':
        """Create DepthAI pipeline."""
        pipeline = dai.Pipeline()

        # === RGB Camera ===
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)  # Preview for streaming
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self.config.fps)

        # Resolution mapping
        resolution_map = {
            "1080p": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "4K": dai.ColorCameraProperties.SensorResolution.THE_4_K,
            "720p": dai.ColorCameraProperties.SensorResolution.THE_720_P,
        }
        cam_rgb.setResolution(
            resolution_map.get(self.config.rgb_resolution,
                             dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        )

        # RGB output
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        # Update dimensions
        self.rgb_width = 640
        self.rgb_height = 480

        # === Stereo Depth ===
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        # Mono camera settings
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Left
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Right

        # Stereo depth configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)

        # Depth thresholds
        config = stereo.initialConfig.get()
        config.postProcessing.thresholdFilter.minRange = self.config.depth_min_mm
        config.postProcessing.thresholdFilter.maxRange = self.config.depth_max_mm
        stereo.initialConfig.set(config)

        # Link mono cameras to stereo
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Depth output
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Update dimensions
        self.depth_width = 640
        self.depth_height = 400

        return pipeline

    def _capture_loop(self):
        """Main capture loop (runs in thread)."""
        while self._running:
            try:
                # Get RGB frame
                rgb_packet = self._rgb_queue.tryGet()
                if rgb_packet is not None:
                    frame = rgb_packet.getCvFrame()
                    with self._lock:
                        self._latest_rgb = frame
                        self._frame_count += 1

                # Get depth frame
                depth_packet = self._depth_queue.tryGet()
                if depth_packet is not None:
                    depth = depth_packet.getFrame()
                    with self._lock:
                        self._latest_depth = depth
                        # Analyze depth sectors
                        self._latest_depth_analysis = self._analyze_depth(depth)

                    if self._frame_callback:
                        self._frame_callback(frame, depth)

            except Exception as e:
                print(f"[OAK-D] Capture error: {e}")

            time.sleep(0.001)  # Prevent tight loop

    def _simulate_capture_loop(self):
        """Simulated capture loop for testing without hardware."""
        while self._running:
            # Generate simulated RGB frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            frame[:, :] = [50, 50, 50]  # Dark gray background

            # Add some simulated features
            import cv2
            cv2.putText(frame, "SIMULATED", (200, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Frame: {self._frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

            # Generate simulated depth map
            # Simulate obstacles at various distances
            depth = np.ones((400, 640), dtype=np.uint16) * 5000  # 5m default

            # Simulate random obstacles
            np.random.seed(int(time.time()) % 1000)
            if np.random.random() > 0.7:
                # Random obstacle on left
                depth[:, 0:200] = np.random.randint(800, 1500)
            if np.random.random() > 0.7:
                # Random obstacle on right
                depth[:, 440:640] = np.random.randint(800, 1500)
            if np.random.random() > 0.8:
                # Random obstacle in center
                depth[:, 200:440] = np.random.randint(500, 1000)

            with self._lock:
                self._latest_rgb = frame
                self._latest_depth = depth
                self._latest_depth_analysis = self._analyze_depth(depth)
                self._frame_count += 1

            if self._frame_callback:
                self._frame_callback(frame, depth)

            time.sleep(1.0 / self.config.fps)

    def _analyze_depth(self, depth_map: np.ndarray) -> DepthAnalysis:
        """
        Analyze depth map by sectors for obstacle avoidance.

        Divides the depth map into left, center, and right sectors
        and calculates average/minimum distances.

        Args:
            depth_map: Raw depth map from stereo camera (mm)

        Returns:
            DepthAnalysis with sector distances and recommendation
        """
        h, w = depth_map.shape

        # Define sector boundaries (horizontal slices)
        left_sector = depth_map[:, 0:w//3]
        center_sector = depth_map[:, w//3:2*w//3]
        right_sector = depth_map[:, 2*w//3:w]

        # Filter out invalid (zero) measurements
        def safe_stats(sector):
            valid = sector[sector > 0]
            if len(valid) == 0:
                return float('inf'), float('inf')
            return np.mean(valid) / 1000.0, np.min(valid) / 1000.0  # Convert mm to m

        left_avg, left_min = safe_stats(left_sector)
        center_avg, center_min = safe_stats(center_sector)
        right_avg, right_min = safe_stats(right_sector)

        # Determine if obstacle detected
        obstacle_detected = center_min < self.config.warning_distance_m

        # Recommend direction based on most open space
        if not obstacle_detected:
            recommended = 'straight'
        elif left_avg > right_avg and left_min > self.config.critical_distance_m:
            recommended = 'left'
        elif right_avg > left_avg and right_min > self.config.critical_distance_m:
            recommended = 'right'
        elif left_avg > right_avg:
            recommended = 'left'
        else:
            recommended = 'right'

        return DepthAnalysis(
            left_avg_distance=left_avg,
            right_avg_distance=right_avg,
            center_avg_distance=center_avg,
            left_min_distance=left_min,
            right_min_distance=right_min,
            center_min_distance=center_min,
            recommended_direction=recommended,
            obstacle_detected=obstacle_detected,
            timestamp=time.time()
        )

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest RGB frame.

        Returns:
            BGR numpy array or None
        """
        with self._lock:
            if self._latest_rgb is not None:
                return self._latest_rgb.copy()
            return None

    def get_depth(self) -> Optional[np.ndarray]:
        """
        Get the latest depth map.

        Returns:
            Depth map in millimeters (uint16) or None
        """
        with self._lock:
            if self._latest_depth is not None:
                return self._latest_depth.copy()
            return None

    def get_frame_and_depth(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get both RGB frame and depth map.

        Returns:
            Tuple of (rgb_frame, depth_map) - either may be None
        """
        with self._lock:
            rgb = self._latest_rgb.copy() if self._latest_rgb is not None else None
            depth = self._latest_depth.copy() if self._latest_depth is not None else None
            return rgb, depth

    def get_depth_analysis(self) -> Optional[DepthAnalysis]:
        """
        Get the latest depth sector analysis.

        Returns:
            DepthAnalysis object or None
        """
        with self._lock:
            return self._latest_depth_analysis

    def analyze_depth_sectors(self) -> Tuple[float, float]:
        """
        Get simplified left/right sector distances.

        Convenience method for obstacle avoidance.

        Returns:
            Tuple of (left_distance, right_distance) in meters
        """
        analysis = self.get_depth_analysis()
        if analysis:
            return analysis.left_avg_distance, analysis.right_avg_distance
        return float('inf'), float('inf')

    def set_frame_callback(self, callback: Callable):
        """
        Set callback for new frames.

        Args:
            callback: Function(rgb_frame, depth_map) called on each new frame
        """
        self._frame_callback = callback

    @property
    def is_running(self) -> bool:
        """Check if camera is running."""
        return self._running

    @property
    def frame_count(self) -> int:
        """Total frames captured."""
        with self._lock:
            return self._frame_count

    def get_stats(self) -> dict:
        """Get camera statistics."""
        analysis = self.get_depth_analysis()
        return {
            'running': self._running,
            'frame_count': self._frame_count,
            'rgb_resolution': f"{self.rgb_width}x{self.rgb_height}",
            'depth_resolution': f"{self.depth_width}x{self.depth_height}",
            'depthai_available': DEPTHAI_AVAILABLE,
            'depth_analysis': {
                'left_distance': analysis.left_avg_distance if analysis else None,
                'center_distance': analysis.center_avg_distance if analysis else None,
                'right_distance': analysis.right_avg_distance if analysis else None,
                'recommended': analysis.recommended_direction if analysis else None,
            } if analysis else None
        }


# Test code
if __name__ == '__main__':
    print("[OAK-D] Testing camera driver...")

    camera = OakDCamera()

    if not camera.start():
        print("[OAK-D] Failed to start camera")
        exit(1)

    try:
        for i in range(100):
            frame = camera.get_frame()
            depth = camera.get_depth()
            analysis = camera.get_depth_analysis()

            if frame is not None and analysis is not None:
                print(f"[{i:3d}] Frame: {frame.shape} | "
                      f"L: {analysis.left_avg_distance:.2f}m | "
                      f"C: {analysis.center_avg_distance:.2f}m | "
                      f"R: {analysis.right_avg_distance:.2f}m | "
                      f"-> {analysis.recommended_direction}")
            else:
                print(f"[{i:3d}] Waiting for frames...")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[OAK-D] Interrupted")
    finally:
        camera.stop()
        print(f"[OAK-D] Stats: {camera.get_stats()}")
