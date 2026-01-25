# Migration vers ROS2 et SLAM Toolbox

Ce document explique comment migrer le projet vers ROS2 pour utiliser des outils industriels comme SLAM Toolbox et Nav2.

## Pourquoi ROS2 ?

| Aspect | Python Pur (Actuel) | ROS2 |
|--------|---------------------|------|
| **Complexité** | Simple | Plus complexe |
| **Écosystème** | Limité | Très riche |
| **SLAM** | BreezySLAM | SLAM Toolbox, Cartographer |
| **Navigation** | Custom | Nav2 (industriel) |
| **Visualisation** | matplotlib | RViz2 (professionnel) |
| **Communauté** | Petite | Énorme (F1Tenth, etc.) |

## Quand migrer ?

Migre vers ROS2 quand :
- [ ] Le prototype Python fonctionne
- [ ] Tu veux des fonctionnalités avancées
- [ ] Tu prépares une compétition (F1Tenth)
- [ ] Tu veux intégrer plus de capteurs

## Installation ROS2 sur Jetson Nano

```bash
# ROS2 Humble pour Ubuntu 22.04 (recommandé)
# ou ROS2 Foxy pour Ubuntu 20.04

# Ajouter les sources ROS2
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Installer ROS2
sudo apt update
sudo apt install ros-humble-desktop  # ou ros-foxy-desktop

# Installer les packages nécessaires
sudo apt install \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    ros-humble-tf2-tools

# Sourcer ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Structure du Workspace ROS2

```
robocar_ws/
├── src/
│   ├── robocar_bringup/          # Launch files
│   │   ├── launch/
│   │   │   ├── robot.launch.py
│   │   │   ├── slam.launch.py
│   │   │   └── navigation.launch.py
│   │   └── config/
│   │       ├── slam_toolbox.yaml
│   │       └── nav2_params.yaml
│   │
│   ├── robocar_description/      # Robot model (URDF)
│   │   └── urdf/
│   │       └── robocar.urdf.xacro
│   │
│   ├── robocar_drivers/          # Hardware drivers
│   │   ├── robocar_drivers/
│   │   │   ├── lidar_node.py
│   │   │   ├── gps_node.py
│   │   │   └── vesc_node.py
│   │   └── package.xml
│   │
│   └── robocar_slam/             # Custom SLAM integration
│       └── ...
│
└── install/                      # Built packages
```

## Configuration SLAM Toolbox

Fichier `config/slam_toolbox.yaml` :

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping  # ou localization

    # Mapper params
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 12.0
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1

    # Pour voiture Ackermann
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000

    # Scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_time_interval: 0.5

    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0

    # Correlation parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
```

## Configuration Nav2

Fichier `config/nav2_params.yaml` :

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    # Pour Ackermann
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    # Pour Ackermann - utiliser Regulated Pure Pursuit
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      max_linear_accel: 1.0
      max_linear_decel: 1.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: 0.05
      use_approach_linear_velocity_scaling: true
      max_allowed_time_to_collision: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

## Launch File Exemple

Fichier `launch/slam.launch.py` :

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robocar_bringup')

    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        # LiDAR driver
        Node(
            package='robocar_drivers',
            executable='lidar_node',
            name='lidar',
            parameters=[{'port': '/dev/ttyUSB0'}]
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
        ),

        # Static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
    ])
```

## Adaptation du Driver LiDAR pour ROS2

```python
#!/usr/bin/env python3
"""ROS2 Node for LD19 LiDAR."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

# Importer notre driver existant
from robocar.drivers.lidar_ld19 import LD19Driver

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # Paramètres
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')

        port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Driver
        self.driver = LD19Driver(port)
        self.driver.start()

        # Timer
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.get_logger().info(f'LiDAR node started on {port}')

    def publish_scan(self):
        scan = self.driver.get_latest_scan()
        if scan is None:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / len(scan.points)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / scan.scan_frequency
        msg.range_min = 0.05
        msg.range_max = 12.0

        msg.ranges = [p.distance if p.valid else float('inf')
                      for p in scan.points]
        msg.intensities = [float(p.intensity) for p in scan.points]

        self.scan_pub.publish(msg)

def main():
    rclpy.init()
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Références F1Tenth

Le projet F1Tenth utilise exactement cette stack et fournit des exemples complets :

### Repositories recommandés

- **[f1tenth_system](https://github.com/f1tenth/f1tenth_system)** - Stack complète F1Tenth
- **[f1tenth_ws (UWaterloo)](https://github.com/CL2-UWaterloo/f1tenth_ws)** - Workspace ROS2 prêt à l'emploi
- **[PhoenixCore](https://github.com/ZhihaoZhang0618/PhoenixCore)** - Stack Nav2 pour F1Tenth
- **[particle_filter](https://github.com/f1tenth/particle_filter)** - Localisation rapide

### Workflow F1Tenth typique

```bash
# 1. Cartographie
ros2 launch slam_toolbox online_async_launch.py

# 2. Sauvegarder la carte
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 3. Localisation
ros2 launch robocar_bringup localization.launch.py map:=~/maps/my_map.yaml

# 4. Navigation
ros2 launch nav2_bringup navigation_launch.py
```

## Comparaison des approches

| Fonctionnalité | Python Pur | ROS2 + SLAM Toolbox |
|----------------|------------|---------------------|
| Installation | `pip install` | Apt + workspace |
| Temps de setup | 30 min | 2-4 heures |
| SLAM | BreezySLAM | SLAM Toolbox |
| Localisation | ParticleFilter custom | AMCL (optimisé) |
| Navigation | À implémenter | Nav2 (complet) |
| Visualisation | matplotlib | RViz2 |
| Débogage | print() | rqt, ros2 topic |
| Performance | Bonne | Excellente |
| Maintenance | Toi | Communauté |

## Conclusion

1. **Commence avec Python pur** (ce qu'on a fait)
2. **Valide les concepts** avec les démos
3. **Migre vers ROS2** quand tu as besoin de :
   - Navigation autonome complète
   - Meilleure performance SLAM
   - Intégration multi-robots
   - Compétitions F1Tenth

La migration sera facile car nos drivers sont déjà structurés de manière compatible avec ROS2.
