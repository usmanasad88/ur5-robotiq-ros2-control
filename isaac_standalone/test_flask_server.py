#!/usr/bin/env python3
"""
Test Flask server for UR5 HTTP control
Provides dummy pose data for testing Isaac Sim integration
"""

from flask import Flask, jsonify
import time
import math
import argparse

app = Flask(__name__)

# Configuration
config = {
    'mode': 'static',  # static, sine, circle
    'amplitude': 0.01,  # meters
    'frequency': 0.5,   # Hz
}


def get_static_pose():
    """Return static small deltas"""
    return {
        "x": 0.0001,
        "y": 0.0001,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "gripper": 0.5
    }


def get_sine_pose(t):
    """Return sinusoidal motion in x-y plane"""
    amp = config['amplitude']
    freq = config['frequency']
    omega = 2 * math.pi * freq
    
    return {
        "x": amp * math.sin(omega * t),
        "y": amp * math.cos(omega * t),
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.001 * math.sin(omega * t),
        "gripper": 0.5 + 0.3 * math.sin(omega * t)
    }


def get_circle_pose(t):
    """Return circular motion"""
    amp = config['amplitude']
    freq = config['frequency']
    omega = 2 * math.pi * freq
    
    return {
        "x": amp * math.cos(omega * t),
        "y": amp * math.sin(omega * t),
        "z": 0.001 * math.sin(2 * omega * t),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.002 * math.sin(omega * t),
        "gripper": 0.5
    }


@app.route('/joint_actions', methods=['GET'])
def joint_actions():
    """Main endpoint that returns target pose"""
    t = time.time()
    
    # Get pose based on mode
    if config['mode'] == 'sine':
        joints = get_sine_pose(t)
    elif config['mode'] == 'circle':
        joints = get_circle_pose(t)
    else:
        joints = get_static_pose()
    
    # Format response
    response = {
        "joint_actions": [
            joints["x"],
            joints["y"],
            joints["z"],
            joints["roll"],
            joints["pitch"],
            joints["yaw"],
            joints["gripper"]
        ],
        "joints": joints,
        "timestamp": t
    }
    
    return jsonify(response)


@app.route('/config', methods=['GET'])
def get_config():
    """Get current configuration"""
    return jsonify(config)


@app.route('/config/<key>/<value>', methods=['POST', 'GET'])
def set_config(key, value):
    """Set configuration parameter"""
    if key in config:
        if key in ['amplitude', 'frequency']:
            config[key] = float(value)
        else:
            config[key] = value
        return jsonify({"status": "ok", "config": config})
    return jsonify({"status": "error", "message": f"Unknown key: {key}"}), 400


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        "status": "ok",
        "timestamp": time.time(),
        "config": config
    })


@app.route('/')
def index():
    """Root endpoint with usage info"""
    return """
    <h1>UR5 HTTP Control Test Server</h1>
    <p>This is a test Flask server for Isaac Sim UR5 HTTP control.</p>
    
    <h2>Endpoints:</h2>
    <ul>
        <li><a href="/joint_actions">/joint_actions</a> - Get target pose (main endpoint)</li>
        <li><a href="/config">/config</a> - Get configuration</li>
        <li>/config/&lt;key&gt;/&lt;value&gt; - Set configuration</li>
        <li><a href="/health">/health</a> - Health check</li>
    </ul>
    
    <h2>Configuration:</h2>
    <ul>
        <li><b>mode</b>: static, sine, circle</li>
        <li><b>amplitude</b>: motion amplitude in meters (default: 0.01)</li>
        <li><b>frequency</b>: motion frequency in Hz (default: 0.5)</li>
    </ul>
    
    <h2>Examples:</h2>
    <ul>
        <li><a href="/config/mode/sine">/config/mode/sine</a> - Enable sinusoidal motion</li>
        <li><a href="/config/mode/circle">/config/mode/circle</a> - Enable circular motion</li>
        <li><a href="/config/amplitude/0.02">/config/amplitude/0.02</a> - Set amplitude to 2cm</li>
        <li><a href="/config/frequency/1.0">/config/frequency/1.0</a> - Set frequency to 1Hz</li>
    </ul>
    
    <h2>Current Configuration:</h2>
    <pre>""" + str(config) + """</pre>
    
    <h2>Testing:</h2>
    <pre>
# Test with curl:
curl http://localhost:5000/joint_actions

# Change mode:
curl http://localhost:5000/config/mode/sine

# Start Isaac Sim:
./run_ur5_http.sh
    </pre>
    """


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test Flask server for UR5 control')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=5000, help='Port to bind to')
    parser.add_argument('--mode', default='static', choices=['static', 'sine', 'circle'],
                        help='Motion mode')
    parser.add_argument('--amplitude', type=float, default=0.01,
                        help='Motion amplitude in meters')
    parser.add_argument('--frequency', type=float, default=0.5,
                        help='Motion frequency in Hz')
    args = parser.parse_args()
    
    # Set initial configuration
    config['mode'] = args.mode
    config['amplitude'] = args.amplitude
    config['frequency'] = args.frequency
    
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║                                                              ║")
    print("║          UR5 HTTP Control Test Server                       ║")
    print("║                                                              ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print("")
    print(f"Mode: {config['mode']}")
    print(f"Amplitude: {config['amplitude']} m")
    print(f"Frequency: {config['frequency']} Hz")
    print("")
    print(f"Server starting at http://{args.host}:{args.port}")
    print("")
    print("Endpoints:")
    print(f"  http://localhost:{args.port}/joint_actions  - Main endpoint")
    print(f"  http://localhost:{args.port}/health         - Health check")
    print(f"  http://localhost:{args.port}/config         - Get config")
    print("")
    print("Press Ctrl+C to stop")
    print("")
    
    app.run(host=args.host, port=args.port, debug=False)
