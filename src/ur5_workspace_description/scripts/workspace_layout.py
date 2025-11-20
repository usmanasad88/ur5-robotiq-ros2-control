"""
Workspace Environment Layout Diagram

This script prints a visual representation of the tabletop workspace
showing the positions of all objects relative to the robot base.

Run: python3 workspace_layout.py
"""

def print_workspace_layout():
    print("\n" + "="*70)
    print(" UR5 WORKSPACE ENVIRONMENT - TOP VIEW ")
    print("="*70)
    print()
    print("                    Y-axis (meters)")
    print("                        ↑")
    print("                   0.4  |")
    print("                        |")
    print("                   0.3  |   🟠 Cylinder")
    print("                        |   (Orange)")
    print("                   0.2  |   🔴 Box 1")
    print("                        |   (Red)")
    print("                   0.1  |")
    print("                        |")
    print("  ←─────────────────────┼─────────────────────→ X-axis")
    print("                   0.0  |🤖 [Robot Base]  🟡 Box 4 (Yellow)")
    print("                        |")
    print("                  -0.1  |          🟢 Box 2 (Green)")
    print("                        |")
    print("                  -0.2  |")
    print("                        |")
    print("                  -0.3  |       🔵 Box 3 (Blue)")
    print("                        |")
    print("                  -0.4  |")
    print("                        ↓")
    print()
    print("  Distance markers (X-axis):")
    print("  0.0    0.1    0.2    0.3    0.4    0.5 (meters)")
    print()
    print("="*70)
    print(" TABLE: 1.2m × 0.8m brown surface at Z = -0.15m ")
    print(" (15cm below robot base, extends across entire workspace)")
    print("="*70)
    print()
    
    print("OBJECT DETAILS:")
    print("-" * 70)
    print()
    
    objects = [
        {
            'name': 'Table',
            'color': 'Brown',
            'type': 'Box',
            'size': '1.2m × 0.8m × 0.1m',
            'position': '[0.0, 0.0, -0.15]',
            'description': 'Main work surface'
        },
        {
            'name': 'Box 1',
            'color': 'Red 🔴',
            'type': 'Cube',
            'size': '0.1m × 0.1m × 0.1m',
            'position': '[0.3, 0.2, -0.05]',
            'description': 'Left side of table'
        },
        {
            'name': 'Box 2',
            'color': 'Green 🟢',
            'type': 'Rectangle',
            'size': '0.12m × 0.08m × 0.15m',
            'position': '[0.35, -0.1, -0.05]',
            'description': 'Center of table'
        },
        {
            'name': 'Box 3',
            'color': 'Blue 🔵',
            'type': 'Tall Box',
            'size': '0.08m × 0.08m × 0.2m',
            'position': '[0.25, -0.25, -0.05]',
            'description': 'Right side of table'
        },
        {
            'name': 'Box 4',
            'color': 'Yellow 🟡',
            'type': 'Flat Box',
            'size': '0.15m × 0.15m × 0.1m',
            'position': '[0.15, 0.0, -0.05]',
            'description': 'Near robot base'
        },
        {
            'name': 'Cylinder',
            'color': 'Orange 🟠',
            'type': 'Cylinder',
            'size': 'H=0.2m, R=0.04m',
            'position': '[0.4, 0.3, 0.0]',
            'description': 'Obstacle on table'
        }
    ]
    
    for obj in objects:
        print(f"  {obj['name']:12} | {obj['color']:15} | {obj['type']:10} | {obj['size']:20}")
        print(f"               | Position: {obj['position']:20} | {obj['description']}")
        print()
    
    print("="*70)
    print()
    print("COORDINATE SYSTEM:")
    print("  • Origin (0,0,0) = Robot base_link")
    print("  • +X = Forward (away from robot)")
    print("  • +Y = Left (from robot's perspective)")
    print("  • +Z = Up")
    print("  • Table surface at Z = -0.10m (relative to base)")
    print("  • All objects sitting on table at Z = -0.05m to -0.10m")
    print()
    print("="*70)
    print()
    print("USAGE:")
    print("  Launch: ros2 launch ur5_workspace_description workspace_environment.launch.py")
    print("  In RViz: Add → MarkerArray → Topic: /workspace_markers")
    print()
    print("="*70)
    print()

if __name__ == "__main__":
    print_workspace_layout()
