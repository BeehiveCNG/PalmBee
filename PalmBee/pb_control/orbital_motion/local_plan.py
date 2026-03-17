#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import signal
import sys

class ScriptRunnerNode(Node):
    def __init__(self):
        super().__init__('script_runner_node')
        
        # Menyimpan subprocess yang dijalankan
        self.processes = []

        # Menjalankan ketiga script secara berurutan
        self.run_scripts()

    def run_scripts(self):
        # Menjalankan script offboard_control.py
        self.get_logger().info('Starting offboard_control.py...')
        self.processes.append(subprocess.Popen(['python3', '/home/ep51lon/polinasi_ws/src/px4_ros_com/src/examples/offboard_py/offboard_control.py']))
        time.sleep(12)  # Tunggu sebentar sebelum melanjutkan

        # Menjalankan script forward.py
        self.get_logger().info('Starting forward.py...')
        forward_process = subprocess.Popen(['python3', '/home/ep51lon/polinasi_ws/src/px4_ros_com/src/examples/offboard_py/forward.py'])
        self.processes.append(forward_process)
        
        # Tunggu beberapa detik sebelum menghentikan forward.py
        time.sleep(10) # Jalankan 10 detik
        self.get_logger().info('Stopping forward.py...')
        forward_process.terminate()  # Menghentikan forward.py
        
        # Menunggu sebentar setelah menghentikan forward.py
        time.sleep(3)

        # Menjalankan script circle.cpp menggunakan ros2 run
        self.get_logger().info('Starting circle.cpp...')
        circle_process = subprocess.Popen(['ros2', 'run', 'px4_ros_com', 'circle'])
        self.processes.append(circle_process)
        
        # Tunggu beberapa detik sebelum menghentikan circle.cpp
        time.sleep(35) # Jalankan 35 detik
        self.get_logger().info('Stopping circle.cpp...')
        circle_process.terminate() # Menghentikan circle.cpp
        
        # Menunggu sebentar setelah menghentikan circle.cpp
        time.sleep(10)
        
        # Menjalankan script landing.py
        self.get_logger().info('Starting landing.py...')
        landing_process = subprocess.Popen(['python3', '/home/ep51lon/polinasi_ws/src/px4_ros_com/src/examples/offboard_py/landing.py'])
        self.processes.append(landing_process)

    def terminate_processes(self):
        # Fungsi untuk menghentikan semua subprocess yang berjalan
        self.get_logger().info('Terminating all processes...')
        for process in self.processes:
            process.terminate()  # Menghentikan subprocess
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ScriptRunnerNode()

    # Menangani sinyal SIGINT (Ctrl+C) untuk menghentikan node dan subprocess
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.terminate_processes()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()