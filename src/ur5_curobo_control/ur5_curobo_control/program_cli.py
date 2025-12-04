#!/usr/bin/env python3
"""
UR5 Program Executor CLI Client

Simple command-line interface for interacting with the UR5 Program Executor Node.
This can be used by AI agents or humans to:
- List available programs
- Load programs
- Execute programs
- Pause/Resume execution
- Stop execution

Usage:
    ros2 run ur5_curobo_control program_cli list
    ros2 run ur5_curobo_control program_cli load pick_and_place_object.prog
    ros2 run ur5_curobo_control program_cli execute
    ros2 run ur5_curobo_control program_cli pause
    ros2 run ur5_curobo_control program_cli resume
    ros2 run ur5_curobo_control program_cli stop
    ros2 run ur5_curobo_control program_cli status
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class ProgramCLIClient(Node):
    def __init__(self):
        super().__init__('program_cli_client')
        
        # Service clients
        self.execute_client = self.create_client(
            Trigger, '/ur5_program_executor/execute_program')
        self.load_client = self.create_client(
            Trigger, '/ur5_program_executor/load_program')
        self.pause_client = self.create_client(
            SetBool, '/ur5_program_executor/pause')
        self.stop_client = self.create_client(
            Trigger, '/ur5_program_executor/stop')
        self.list_client = self.create_client(
            Trigger, '/ur5_program_executor/list_programs')
        
        # Parameter client for setting program file
        self.param_client = self.create_client(
            SetParameters, '/ur5_program_executor/set_parameters')
        
        # Status subscriber
        self.last_status = None
        self.status_sub = self.create_subscription(
            String, '/ur5_program_executor/status',
            self.status_callback, 10)
    
    def status_callback(self, msg):
        self.last_status = msg.data
    
    def wait_for_service(self, client, timeout_sec=5.0):
        """Wait for a service to become available."""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return False
        return True
    
    def call_trigger_service(self, client, description):
        """Call a Trigger service and print result."""
        if not self.wait_for_service(client):
            return False
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                print(f"✓ {description}: {result.message}")
            else:
                print(f"✗ {description} failed: {result.message}")
            return result.success
        else:
            print(f"✗ {description}: Service call failed")
            return False
    
    def list_programs(self):
        """List available programs."""
        return self.call_trigger_service(self.list_client, "List programs")
    
    def load_program(self, program_file):
        """Load a program file."""
        # First, set the program_file parameter
        if not self.wait_for_service(self.param_client):
            return False
        
        param = Parameter()
        param.name = 'program_file'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = program_file
        
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            print(f"✗ Failed to set program file parameter")
            return False
        
        # Now call the load service
        return self.call_trigger_service(self.load_client, f"Load program '{program_file}'")
    
    def execute_program(self):
        """Execute the loaded program."""
        return self.call_trigger_service(self.execute_client, "Execute program")
    
    def pause_execution(self):
        """Pause execution."""
        if not self.wait_for_service(self.pause_client):
            return False
        
        request = SetBool.Request()
        request.data = True
        future = self.pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            print(f"✓ Execution paused")
            return True
        print("✗ Pause failed")
        return False
    
    def resume_execution(self):
        """Resume execution."""
        if not self.wait_for_service(self.pause_client):
            return False
        
        request = SetBool.Request()
        request.data = False
        future = self.pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            print(f"✓ Execution resumed")
            return True
        print("✗ Resume failed")
        return False
    
    def stop_execution(self):
        """Stop execution."""
        return self.call_trigger_service(self.stop_client, "Stop execution")
    
    def get_status(self):
        """Get current status."""
        # Spin briefly to receive status
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_status:
                print(f"Status: {self.last_status}")
                return True
        print("No status received (is the executor node running?)")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='UR5 Program Executor CLI Client',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  List available programs:
    ros2 run ur5_curobo_control program_cli list
    
  Load and execute a program:
    ros2 run ur5_curobo_control program_cli load pick_and_place_object.prog
    ros2 run ur5_curobo_control program_cli execute
    
  Control execution:
    ros2 run ur5_curobo_control program_cli pause
    ros2 run ur5_curobo_control program_cli resume
    ros2 run ur5_curobo_control program_cli stop
    
  Check status:
    ros2 run ur5_curobo_control program_cli status
""")
    
    parser.add_argument('command', choices=['list', 'load', 'execute', 'pause', 
                                            'resume', 'stop', 'status'],
                       help='Command to execute')
    parser.add_argument('program', nargs='?', default=None,
                       help='Program file name (required for load command)')
    
    args = parser.parse_args()
    
    rclpy.init()
    client = ProgramCLIClient()
    
    try:
        if args.command == 'list':
            client.list_programs()
        elif args.command == 'load':
            if not args.program:
                print("Error: program name required for load command")
                print("Usage: program_cli load <program_file>")
                sys.exit(1)
            client.load_program(args.program)
        elif args.command == 'execute':
            client.execute_program()
        elif args.command == 'pause':
            client.pause_execution()
        elif args.command == 'resume':
            client.resume_execution()
        elif args.command == 'stop':
            client.stop_execution()
        elif args.command == 'status':
            client.get_status()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
