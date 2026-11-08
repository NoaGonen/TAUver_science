#!/usr/bin/env python3
import subprocess
import time
import sys

def main():
    serial_port = '/dev/ttyACM0'
    baud_rate = '115200'
    ros_distro = 'jazzy'  # Matches your container image

    print(f"[*] Starting micro-ROS Agent container on {serial_port}...")
    
    # 1. Run the micro-ROS Agent as a background container (-d)
    agent_cmd = [
        'sudo', 'docker', 'run', '-d', '--rm',
        '--name', 'microros_agent',
        '--net=host',
        f'--device={serial_port}',
        f'microros/micro-ros-agent:{ros_distro}',
        'serial', '--dev', serial_port, '-b', baud_rate
    ]
    
    try:
        subprocess.run(agent_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[!] Error starting micro-ROS Agent. Is the Nucleo plugged into {serial_port}?")
        sys.exit(1)

    # Give the agent a moment to claim the serial port
    time.sleep(2)

    print("[*] Starting ROS 2 listener container (Press Ctrl+C to exit)...")
    
    # 2. Run a ROS 2 container that executes an inline Python script to listen to the topic
    listener_cmd = [
        'sudo', 'docker', 'run', '--rm',
        '--name', 'tauver_listener',
        '--net=host',
        f'ros:{ros_distro}',
        'python3', '-c', 
        "import rclpy\n"
        "from rclpy.node import Node\n"
        "from std_msgs.msg import String\n\n"
        "class TauverListener(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('laptop_listener_node')\n"
        "        self.subscription = self.create_subscription(\n"
        "            String, 'tauver/telemetry', self.listener_callback, 10)\n"
        "        self.get_logger().info('Listening for telemetry on /tauver/telemetry...')\n\n"
        "    def listener_callback(self, msg):\n"
        "        print(f'[NUCLEO DATA] {msg.data}')\n\n"
        "def main():\n"
        "    rclpy.init()\n"
        "    node = TauverListener()\n"
        "    try:\n"
        "        rclpy.spin(node)\n"
        "    except KeyboardInterrupt:\n"
        "        pass\n"
        "    finally:\n"
        "        node.destroy_node()\n"
        "        rclpy.shutdown()\n\n"
        "if __name__ == '__main__':\n"
        "    main()"
    ]

    try:
        # This blocks and streams the listener output to your terminal
        subprocess.run(listener_cmd)
    except KeyboardInterrupt:
        print("\n[*] Shutting down...")
    finally:
        # Clean shutdown: stop the background micro-ROS agent container
        print("[*] Stopping micro-ROS Agent container...")
        subprocess.run(['sudo', 'docker', 'stop', 'microros_agent'], 
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[*] Cleanup complete. Goodbye!")

if __name__ == '__main__':
    main()