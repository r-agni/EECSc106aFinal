"""
Test installation and configuration of path planning system.

Run this script to verify all components are working without needing a drone.
"""

import sys
import os
import numpy as np
from pathlib import Path

# Add parent directory to path so we can import path_planning
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

def test_imports():
    """Test that all required modules can be imported"""
    print("\n" + "="*60)
    print("Testing Imports...")
    print("="*60)

    try:
        import cv2
        print(f"[OK] OpenCV version: {cv2.__version__}")
    except ImportError as e:
        print(f"[FAIL] OpenCV import failed: {e}")
        return False

    try:
        import numpy
        print(f"[OK] NumPy version: {numpy.__version__}")
    except ImportError as e:
        print(f"[FAIL] NumPy import failed: {e}")
        return False

    try:
        import matplotlib
        print(f"[OK] Matplotlib version: {matplotlib.__version__}")
    except ImportError as e:
        print(f"[FAIL] Matplotlib import failed: {e}")
        return False

    try:
        from dotenv import load_dotenv
        print(f"[OK] python-dotenv installed")
    except ImportError as e:
        print(f"[FAIL] python-dotenv import failed: {e}")
        return False

    return True


def test_path_planning_modules():
    """Test that all path_planning modules can be imported"""
    print("\n" + "="*60)
    print("Testing Path Planning Modules...")
    print("="*60)

    modules = [
        'config',
        'utils',
        'aruco_detector',
        'search_planner',
        'movement_recorder',
        'path_optimizer',
        'visualizer',
        'mission_controller'
    ]

    for module_name in modules:
        try:
            module = __import__('path_planning.' + module_name, fromlist=[module_name])
            print(f"[OK] {module_name}")
        except ImportError as e:
            print(f"[FAIL] {module_name}: {e}")
            return False

    return True


def test_configuration():
    """Test configuration loading"""
    print("\n" + "="*60)
    print("Testing Configuration...")
    print("="*60)

    try:
        from path_planning import PathPlanningConfig

        config = PathPlanningConfig()

        print(f"[OK] Configuration loaded successfully")
        print(f"  Search area: {config.SEARCH_AREA_WIDTH_M}m x {config.SEARCH_AREA_HEIGHT_M}m")
        print(f"  Grid spacing: {config.GRID_CELL_SIZE_M}m")
        print(f"  ArUco dict: {config.ARUCO_DICT}")
        print(f"  Tag sizes: {config.TAG_0_SIZE_M}m, {config.TAG_1_SIZE_M}m")

        return True
    except Exception as e:
        print(f"[FAIL] Configuration failed: {e}")
        return False


def test_aruco_detector():
    """Test ArUco detector initialization"""
    print("\n" + "="*60)
    print("Testing ArUco Detector...")
    print("="*60)

    try:
        from path_planning import ArucoDetector, PathPlanningConfig

        config = PathPlanningConfig()
        detector = ArucoDetector(config)

        print(f"[OK] ArUco detector initialized")
        print(f"  Dictionary: {config.ARUCO_DICT}")
        print(f"  Marker sizes: {detector.marker_sizes}")

        # Test with dummy frame
        dummy_frame = np.zeros((240, 320, 3), dtype=np.uint8)
        dummy_pos = {'x': 0, 'y': 0, 'z': 120}
        dummy_yaw = 0

        tags = detector.detect_tags(dummy_frame, dummy_pos, dummy_yaw)
        print(f"[OK] Detection on dummy frame: {len(tags)} tags (expected 0)")

        return True
    except Exception as e:
        print(f"[FAIL] ArUco detector failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_search_planner():
    """Test search planner waypoint generation"""
    print("\n" + "="*60)
    print("Testing Search Planner...")
    print("="*60)

    try:
        from path_planning import SearchPlanner, PathPlanningConfig

        config = PathPlanningConfig()
        planner = SearchPlanner(config)

        waypoints = planner.generate_grid_waypoints()
        print(f"[OK] Generated {len(waypoints)} waypoints")

        waypoints_with_scan = planner.generate_waypoints_with_rotation_scan()
        print(f"[OK] Generated {len(waypoints_with_scan)} waypoints with rotation scan")

        estimated_time = planner.estimate_exploration_time(waypoints_with_scan)
        print(f"[OK] Estimated exploration time: {estimated_time:.1f}s ({estimated_time/60:.1f} min)")

        return True
    except Exception as e:
        print(f"[FAIL] Search planner failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_movement_recorder():
    """Test movement recorder"""
    print("\n" + "="*60)
    print("Testing Movement Recorder...")
    print("="*60)

    try:
        from path_planning import MovementRecorder, PathPlanningConfig

        config = PathPlanningConfig()
        recorder = MovementRecorder(config)

        # Simulate recording
        recorder.start_mission(5.0, 5.0, 100)

        recorder.record_movement_start(
            'move_forward',
            {'distance': 100},
            {'x': 0, 'y': 0, 'z': 120},
            0
        )

        recorder.record_movement_end(
            True,
            {'x': 100, 'y': 0, 'z': 120},
            0,
            2.1
        )

        recorder.end_mission(95)

        log = recorder.get_movement_log()
        print(f"[OK] Recorded {len(log['movements'])} movements")
        print(f"[OK] Mission ID: {log['mission_id']}")

        return True
    except Exception as e:
        print(f"[FAIL] Movement recorder failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_path_optimizer():
    """Test path optimizer"""
    print("\n" + "="*60)
    print("Testing Path Optimizer...")
    print("="*60)

    try:
        from path_planning import PathOptimizer, PathPlanningConfig

        config = PathPlanningConfig()
        optimizer = PathOptimizer(config)

        # Create dummy movement log
        movement_log = {
            'search_area': {'width_m': 5.0, 'height_m': 5.0},
            'movements': [],
            'obstacles_encountered': []
        }

        start_pos = {'x': 0, 'y': 0, 'z': 120}
        goal_pos = {'x': 300, 'y': 300, 'z': 120}

        optimal_path = optimizer.calculate_optimal_path(start_pos, goal_pos, movement_log)

        if optimal_path:
            print(f"[OK] Generated optimal path with {len(optimal_path['waypoints'])} waypoints")
            print(f"[OK] Distance: {optimal_path['total_distance_cm']:.0f} cm")
            print(f"[OK] Estimated time: {optimal_path['estimated_time_sec']:.1f}s")
        else:
            print(f"[FAIL] Path optimization returned None")
            return False

        return True
    except Exception as e:
        print(f"[FAIL] Path optimizer failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_visualizer():
    """Test visualizer (without showing plot)"""
    print("\n" + "="*60)
    print("Testing Visualizer...")
    print("="*60)

    try:
        from path_planning import PathVisualizer, PathPlanningConfig

        config = PathPlanningConfig()
        # Disable live plot for testing
        config.SHOW_LIVE_PLOT = False

        visualizer = PathVisualizer(config)
        print(f"[OK] Visualizer initialized")

        return True
    except Exception as e:
        print(f"[FAIL] Visualizer failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests"""
    print("\n" + "="*60)
    print("  PATH PLANNING SYSTEM - INSTALLATION TEST")
    print("="*60)

    tests = [
        ("Imports", test_imports),
        ("Path Planning Modules", test_path_planning_modules),
        ("Configuration", test_configuration),
        ("ArUco Detector", test_aruco_detector),
        ("Search Planner", test_search_planner),
        ("Movement Recorder", test_movement_recorder),
        ("Path Optimizer", test_path_optimizer),
        ("Visualizer", test_visualizer),
    ]

    results = []

    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"\n[FAIL] {test_name} crashed: {e}")
            results.append((test_name, False))

    # Summary
    print("\n" + "="*60)
    print("  TEST SUMMARY")
    print("="*60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "[OK] PASS" if result else "[FAIL] FAIL"
        print(f"{status}: {test_name}")

    print("\n" + "="*60)
    print(f"  {passed}/{total} tests passed")
    print("="*60)

    if passed == total:
        print("\n[OK] All tests passed! Path planning system is ready to use.")
        print("\nNext steps:")
        print("1. Print ArUco tags (see QUICK_START.md)")
        print("2. Configure search area in path_planning/.env")
        print("3. Run: python path_planning/example_usage.py")
        return 0
    else:
        print("\n[FAIL] Some tests failed. Please fix the errors above.")
        print("\nCommon fixes:")
        print("- Install missing dependencies: pip install -r path_planning/requirements.txt")
        print("- Ensure .env file exists in path_planning/")
        return 1


if __name__ == "__main__":
    sys.exit(main())
