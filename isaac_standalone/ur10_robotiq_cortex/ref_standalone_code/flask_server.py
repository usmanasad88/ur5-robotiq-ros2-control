#!/usr/bin/env python3
# For context only. Actual script is running elsewhere

import os
from flask import Flask, jsonify, request
import time
from octo_inference import inference_engine, get_joint_actions

os.environ['TOKENIZERS_PARALLELISM'] = 'false'

app = Flask(__name__)

# Global variables to store the latest joint actions
latest_joint_actions = None
latest_timestamp = None

@app.route('/joint_actions', methods=['GET'])
def get_joint_actions_endpoint():
    if latest_joint_actions is None:
        return jsonify({"error": "No joint actions available"}), 404
    
    return jsonify({
        "joint_actions": latest_joint_actions,
        "timestamp": latest_timestamp,
        "joints": {
            "x": latest_joint_actions[0],
            "y": latest_joint_actions[1], 
            "z": latest_joint_actions[2],
            "roll": latest_joint_actions[3],
            "pitch": latest_joint_actions[4],
            "yaw": latest_joint_actions[5],
            "gripper": latest_joint_actions[6]
        }
    })

@app.route('/inference', methods=['POST'])
def run_new_inference():
    global latest_joint_actions, latest_timestamp
    
    data = request.get_json()
    task_text = data.get('task', None)
    image_path = data.get('image_path', None)
    
    # Get joint actions using the inference module
    result = get_joint_actions(task_text, image_path)
    
    # Update global variables
    latest_joint_actions = result["joint_actions"]
    latest_timestamp = result["timestamp"]
    
    return jsonify(result)

@app.route('/status', methods=['GET'])
def get_status():
    config = inference_engine.get_default_config()
    return jsonify({
        "status": "running",
        "model_loaded": config["model_loaded"],
        "latest_inference": latest_timestamp,
        "default_task": config["default_task"],
        "default_image_path": config["default_image_path"]
    })

def initialize_and_run_inference():
    global latest_joint_actions, latest_timestamp
    
    # Load the model
    inference_engine.load_model()
    
    # Run initial inference with defaults
    result = get_joint_actions()
    latest_joint_actions = result["joint_actions"]
    latest_timestamp = result["timestamp"]

if __name__ == "__main__":
    initialize_and_run_inference()
    app.run(host='0.0.0.0', port=5000, debug=False)
