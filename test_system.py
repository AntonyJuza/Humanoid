#!/usr/bin/env python3
"""
Test script for humanoid robot system
Run this to verify all components are working
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time
import sys


class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # Subscribers for status checking
        self.odom_received = False
        self.joints_received = False
        self.vlm_ready = False
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.joints_sub = self.create_subscription(
            JointState, 'joint_states', self.joints_callback, 10)
        self.vlm_status_sub = self.create_subscription(
            Bool, 'vlm_ready', self.vlm_status_callback, 10)
        
        # Publishers for testing
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vlm_input_pub = self.create_publisher(String, 'vlm_text_input', 10)
        
        self.get_logger().info('System Tester initialized')
    
    def odom_callback(self, msg):
        self.odom_received = True
    
    def joints_callback(self, msg):
        self.joints_received = True
    
    def vlm_status_callback(self, msg):
        self.vlm_ready = msg.data
    
    def test_motor_control(self):
        """Test basic motor control"""
        self.get_logger().info('Testing motor control...')
        
        # Wait for odometry
        start_time = time.time()
        while not self.odom_received and time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.odom_received:
            self.get_logger().error('❌ Odometry not received - motor control may not be running')
            return False
        
        # Wait for joint states
        start_time = time.time()
        while not self.joints_received and time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joints_received:
            self.get_logger().error('❌ Joint states not received - encoders may not be working')
            return False
        
        # Test forward movement
        self.get_logger().info('Testing forward movement...')
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        time.sleep(2.0)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info('✓ Motor control test passed')
        return True
    
    def test_vlm_system(self):
        """Test VLM integration"""
        self.get_logger().info('Testing VLM system...')
        
        # Wait for VLM ready status
        start_time = time.time()
        while not self.vlm_ready and time.time() - start_time < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.vlm_ready:
            self.get_logger().warn('⚠ VLM not ready - may not be running or model not loaded')
            return False
        
        # Send test query
        self.get_logger().info('Sending test query to VLM...')
        msg = String()
        msg.data = 'Hello, can you see me?'
        self.vlm_input_pub.publish(msg)
        
        self.get_logger().info('✓ VLM system appears to be running')
        return True
    
    def run_tests(self):
        """Run all system tests"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting System Tests')
        self.get_logger().info('=' * 50)
        
        results = {}
        
        # Test 1: Motor Control
        results['Motor Control'] = self.test_motor_control()
        time.sleep(1.0)
        
        # Test 2: VLM System
        results['VLM System'] = self.test_vlm_system()
        
        # Print results
        self.get_logger().info('=' * 50)
        self.get_logger().info('Test Results:')
        self.get_logger().info('=' * 50)
        
        all_passed = True
        for test_name, passed in results.items():
            status = '✓ PASSED' if passed else '❌ FAILED'
            self.get_logger().info(f'{test_name}: {status}')
            if not passed:
                all_passed = False
        
        self.get_logger().info('=' * 50)
        
        if all_passed:
            self.get_logger().info('All tests passed! System is ready.')
        else:
            self.get_logger().warn('Some tests failed. Check the logs above.')
        
        return all_passed


def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()
    
    try:
        # Give nodes time to start up
        time.sleep(2.0)
        
        # Run tests
        success = tester.run_tests()
        
        # Exit with appropriate code
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()