# Kinesthetic Teaching System Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                     UR5 Kinesthetic Teaching System                 │
└─────────────────────────────────────────────────────────────────────┘

                              ┌──────────────┐
                              │   UR5 Robot  │
                              │  Controller  │
                              └──────┬───────┘
                                     │
                    ┌────────────────┴────────────────┐
                    │                                 │
              ┌─────▼─────┐                    ┌─────▼─────┐
              │ /joint_   │                    │    TF2    │
              │  states   │                    │  (tool0)  │
              └─────┬─────┘                    └─────┬─────┘
                    │                                │
                    │                                │
        ┌───────────┴────────────────────────────────┴──────────┐
        │                                                        │
        │         TRAJECTORY RECORDER NODE                       │
        │                                                        │
        │  ┌──────────────┐         ┌──────────────┐           │
        │  │ Joint Space  │         │ Task Space   │           │
        │  │   Recorder   │         │   Recorder   │           │
        │  │              │         │              │           │
        │  │ • Positions  │         │ • Poses      │           │
        │  │ • Velocities │         │ • Timestamps │           │
        │  │ • Timestamps │         │              │           │
        │  └──────┬───────┘         └──────┬───────┘           │
        │         │                        │                    │
        │         └──────────┬─────────────┘                    │
        │                    │                                  │
        │           ┌────────▼────────┐                         │
        │           │  JSON Trajectory │                        │
        │           │      File        │                        │
        │           │ ~/ur5_trajectories/                       │
        │           │ trajectory_*.json│                        │
        │           └────────┬─────────┘                        │
        └────────────────────┼──────────────────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
    ┌─────────▼────────┐         ┌─────────▼─────────┐
    │   VISUALIZER     │         │    PLAYER         │
    │      NODE        │         │     NODE          │
    │                  │         │                   │
    │ ┌──────────────┐ │         │ ┌───────────────┐ │
    │ │ RViz Markers │ │         │ │ Joint Traj.   │ │
    │ │              │ │         │ │ Action Client │ │
    │ │ • Path       │ │         │ └───────┬───────┘ │
    │ │ • Waypoints  │ │         │         │         │
    │ │ • Start/End  │ │         │         │         │
    │ │ • Arrows     │ │         │         │         │
    │ └──────┬───────┘ │         │         │         │
    │        │         │         │         │         │
    └────────┼─────────┘         └─────────┼─────────┘
             │                             │
             │                             │
        ┌────▼────┐                   ┌────▼────┐
        │  RViz   │                   │  Joint  │
        │ Display │                   │Trajectory│
        └─────────┘                   │Controller│
                                      └─────┬────┘
                                            │
                                      ┌─────▼─────┐
                                      │ UR5 Robot │
                                      │  Motion   │
                                      └───────────┘
```

## Data Flow

### Recording Phase

```
Manual Robot Movement
        │
        ▼
┌───────────────────┐
│  Joint States     │ ──► Joint Positions, Velocities
│  Publisher        │
└───────────────────┘
        │
        ▼
┌───────────────────┐
│  TF Transform     │ ──► End-Effector Pose
│  Lookup           │
└───────────────────┘
        │
        ▼
┌───────────────────┐
│  Trajectory       │ ──► Record if change > threshold
│  Recorder         │
└───────────────────┘
        │
        ▼
┌───────────────────┐
│  JSON File        │ ──► Save both joint & task space
│  ~/trajectories/  │
└───────────────────┘
```

### Playback Phase

```
┌───────────────────┐
│  JSON File        │ ──► Load trajectory data
│  ~/trajectories/  │
└───────────────────┘
        │
        ▼
┌───────────────────┐
│  Trajectory       │ ──► Parse joint positions
│  Player           │      Calculate time points
└───────────────────┘
        │
        ▼
┌───────────────────┐
│ FollowJointTraj   │ ──► Execute motion
│ Action Goal       │
└───────────────────┘
        │
        ▼
┌───────────────────┐
│  Controller       │ ──► Move robot
│  scaled_joint_... │
└───────────────────┘
        │
        ▼
   Robot Motion
```

## Service Interaction Diagram

```
┌──────────────┐
│     USER     │
└──────┬───────┘
       │
       │ ros2 service call /trajectory_recorder/start_recording
       ▼
┌──────────────────┐
│   RECORDER       │
│   [Recording]    │
└──────┬───────────┘
       │
       │ ros2 service call /trajectory_recorder/stop_recording
       ▼
┌──────────────────┐
│   RECORDER       │
│   [Saving...]    │
└──────┬───────────┘
       │
       │ Trajectory saved ✓
       ▼
┌──────────────────┐
│     USER         │
└──────┬───────────┘
       │
       │ ros2 service call /trajectory_player/load_trajectory
       ▼
┌──────────────────┐
│    PLAYER        │
│   [Loading...]   │
└──────┬───────────┘
       │
       │ Trajectory loaded ✓
       ▼
┌──────────────────┐
│     USER         │
└──────┬───────────┘
       │
       │ ros2 service call /trajectory_player/play_trajectory
       ▼
┌──────────────────┐
│    PLAYER        │
│   [Playing...]   │
└──────┬───────────┘
       │
       │ Send action goal
       ▼
┌──────────────────┐
│   CONTROLLER     │
│   [Executing]    │
└──────┬───────────┘
       │
       │ Robot moves!
       ▼
    Complete ✓
```

## Topic Flow

```
/joint_states (sensor_msgs/JointState)
    │
    └──► Trajectory Recorder
            │
            ├──► /trajectory_recorder/recording_status (std_msgs/String)
            │
            └──► /trajectory_recorder/trajectory_markers (MarkerArray)
                     │
                     └──► RViz Display

/tf (tf2_msgs/TFMessage)
    │
    └──► Trajectory Recorder (via TF Buffer)

/trajectory_player/playback_status (std_msgs/String)
    │
    └──► Status Monitor

/trajectory_player/set_playback_speed (std_msgs/Float32)
    │
    └──► Trajectory Player

/trajectory_visualizer/trajectory_visualization (MarkerArray)
    │
    └──► RViz Display
```

## File System Layout

```
~/ur5_trajectories/
├── trajectory_20251119_140000.json
├── trajectory_20251119_141500.json
├── trajectory_20251119_143000.json
└── ...

Each JSON file contains:
{
  "name": "trajectory_20251119_140000",
  "timestamp": "2025-11-19T14:00:00",
  "joint_space": {
    "joint_names": [...],
    "positions": [...],
    "velocities": [...],
    "timestamps": [...]
  },
  "task_space": {
    "poses": [...],
    "timestamps": [...]
  }
}
```

## Node Communication

```
┌─────────────────────┐         ┌─────────────────────┐
│ trajectory_recorder │         │ trajectory_player   │
│                     │         │                     │
│ Services:           │         │ Services:           │
│ • start_recording   │         │ • load_trajectory   │
│ • stop_recording    │         │ • play_trajectory   │
│ • save_trajectory   │         │ • stop_playback     │
│ • clear_trajectory  │         │ • list_trajectories │
│                     │         │                     │
│ Publishers:         │         │ Publishers:         │
│ • recording_status  │         │ • playback_status   │
│ • trajectory_markers│         │                     │
│                     │         │                     │
│ Subscribers:        │         │ Subscribers:        │
│ • /joint_states     │         │ • set_playback_speed│
│ • /tf               │         │                     │
│                     │         │                     │
│ Action Clients:     │         │ Action Clients:     │
│ • none              │         │ • FollowJointTraj   │
└─────────────────────┘         └─────────────────────┘

┌─────────────────────┐
│trajectory_visualizer│
│                     │
│ Services:           │
│ • load_latest       │
│ • clear_visual...   │
│                     │
│ Publishers:         │
│ • trajectory_visual │
│                     │
│ Subscribers:        │
│ • none              │
└─────────────────────┘
```

## Typical Workflow Timeline

```
Time  │ Action                          │ Node        │ State
──────┼─────────────────────────────────┼─────────────┼──────────
0:00  │ Launch system                   │ All         │ Idle
0:05  │ call start_recording            │ Recorder    │ Recording
0:10  │ Move robot manually             │ Recorder    │ Recording
0:45  │ call stop_recording             │ Recorder    │ Saving
0:46  │ File saved                      │ Recorder    │ Idle
1:00  │ call load_trajectory            │ Player      │ Loading
1:01  │ Trajectory loaded               │ Player      │ Ready
1:05  │ call play_trajectory            │ Player      │ Playing
1:45  │ Playback complete               │ Player      │ Idle
2:00  │ call load_latest                │ Visualizer  │ Visualizing
2:01  │ View in RViz                    │ Visualizer  │ Active
```
