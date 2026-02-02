#!/usr/bin/env python3
"""
Test rapide de l'architecture unifiee.

Verifie que tous les modules s'importent correctement
et que les interfaces fonctionnent.
"""

import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))


def test_imports():
    """Test des imports."""
    print("=== Test des imports ===\n")

    # Interfaces
    print("1. Interfaces...")
    try:
        from interface import (
            ILidarSensor, IGPSSensor, IMotorController,
            LidarData, LidarPoint, GPSData,
            RealLidarAdapter, RealGPSAdapter, RealMotorAdapter,
            SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
        )
        print("   OK - Toutes les interfaces importees")
    except ImportError as e:
        print(f"   ERREUR: {e}")
        return False

    # Simulation
    print("2. Simulation...")
    try:
        from simulation import (
            LidarSimulator, GPSSimulator, Environment,
            create_parking_env, create_corridor_env, create_epitech_env
        )
        print("   OK - Modules de simulation importes")
    except ImportError as e:
        print(f"   ERREUR: {e}")
        return False

    # Navigation controller
    print("3. Navigation controller...")
    try:
        from navigation_controller import NavigationController, NavigationState, Waypoint
        print("   OK - Navigation controller importe")
    except ImportError as e:
        print(f"   ERREUR: {e}")
        return False

    # Drivers (peut echouer si pas de hardware)
    print("4. Drivers...")
    try:
        from driver import LidarDriver, GPSDriver, VESCController
        print("   OK - Drivers importes (hardware requis pour execution)")
    except ImportError as e:
        print(f"   WARNING: {e} (normal si pas de hardware)")

    return True


def test_simulation_adapters():
    """Test des adaptateurs de simulation."""
    print("\n=== Test des adaptateurs simulation ===\n")

    from simulation import create_epitech_env
    from interface import (
        SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
    )

    # Creer environnement
    print("1. Creation environnement Epitech...")
    env = create_epitech_env()
    print(f"   Environnement: {env.width}x{env.height}m, {len(env.obstacles)} obstacles")

    # Test LiDAR adapter
    print("2. Test LiDAR adapter...")
    lidar = SimulatedLidarAdapter(env)
    if lidar.start():
        lidar.set_robot_pose(0, 0, 0)
        scan = lidar.get_scan()
        if scan:
            valid_points = len([p for p in scan.points if p.valid])
            print(f"   OK - {valid_points} points valides")
        else:
            print("   WARNING - Pas de scan")
        lidar.stop()
    else:
        print("   ERREUR - Demarrage echoue")
        return False

    # Test GPS adapter
    print("3. Test GPS adapter...")
    gps = SimulatedGPSAdapter(origin_lat=48.8156, origin_lon=2.3631)
    if gps.start():
        gps.set_local_position(10, 5, 0.5)
        pos = gps.get_position()
        if pos:
            print(f"   OK - Position: ({pos.latitude:.6f}, {pos.longitude:.6f})")
            print(f"        Qualite: {pos.quality_string}, Precision: {pos.accuracy_h}m")
        else:
            print("   WARNING - Pas de position")
        gps.stop()
    else:
        print("   ERREUR - Demarrage echoue")
        return False

    # Test Motor adapter
    print("4. Test Motor adapter...")
    motor = SimulatedMotorAdapter(wheelbase=0.26)
    if motor.start():
        motor.set_pose(0, 0, 0)
        motor.set_speed(0.5)
        motor.set_steering(0.2)

        # Simuler quelques pas
        for _ in range(10):
            motor.update_physics(0.1)

        x, y, theta = motor.get_pose()
        print(f"   OK - Apres 1s: position=({x:.2f}, {y:.2f}), theta={theta:.2f}rad")
        motor.stop()
    else:
        print("   ERREUR - Demarrage echoue")
        return False

    return True


def test_navigation_controller():
    """Test du controleur de navigation."""
    print("\n=== Test Navigation Controller ===\n")

    from simulation import create_epitech_env
    from interface import (
        SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
    )
    from navigation_controller import NavigationController

    # Creer les composants
    env = create_epitech_env()
    lidar = SimulatedLidarAdapter(env)
    gps = SimulatedGPSAdapter(origin_lat=48.8156, origin_lon=2.3631)
    motor = SimulatedMotorAdapter(wheelbase=0.26)

    # Creer le controleur
    print("1. Creation du controleur...")
    controller = NavigationController(
        lidar=lidar,
        gps=gps,
        motor=motor,
        mode='simulation'
    )
    print("   OK")

    # Configurer
    print("2. Configuration...")
    controller.set_initial_pose(0, 0, 0)
    controller.add_waypoint_local(5, 0)
    controller.add_waypoint_local(10, 5)
    print(f"   {len(controller.waypoints)} waypoints configures")

    # Initialiser capteurs
    print("3. Initialisation capteurs...")
    if not controller.init_sensors():
        print("   ERREUR - Init capteurs echouee")
        return False
    print("   OK")

    # Executer quelques iterations
    print("4. Test navigation (10 iterations)...")
    for i in range(10):
        controller.update_state_from_motor(0.1)
        result = controller.navigation_step()

    print(f"   Position finale: ({controller.state.x:.2f}, {controller.state.y:.2f})")
    print(f"   Distance parcourue: {controller.state.total_distance:.2f}m")

    # Shutdown
    controller.shutdown()
    print("   OK - Navigation testee")

    return True


def main():
    """Point d'entree."""
    print("=" * 60)
    print("   TEST ARCHITECTURE ROBOCAR")
    print("=" * 60)

    all_passed = True

    # Test imports
    if not test_imports():
        all_passed = False

    # Test adaptateurs
    if not test_simulation_adapters():
        all_passed = False

    # Test navigation controller
    if not test_navigation_controller():
        all_passed = False

    # Resume
    print("\n" + "=" * 60)
    if all_passed:
        print("   TOUS LES TESTS PASSES!")
    else:
        print("   CERTAINS TESTS ONT ECHOUE")
    print("=" * 60)

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
