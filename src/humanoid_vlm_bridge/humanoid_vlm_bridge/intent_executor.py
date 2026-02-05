#!/usr/bin/env python3
"""
Intent Executor - Converts VLM intents into robot actions

This node subscribes to VLM intent messages and publishes appropriate
commands to control the robot (movement, gestures, etc.)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import json
from typing import Dict, Optional


class IntentExecutor(Node):
    """Execute robot actions based on VLM intents"""
    
    def __init__(self):
        super().__init__('intent_executor')
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.3)  # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('action_duration', 2.0)  # seconds
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.action_duration = self.get_parameter('action_duration').value
        
        # Current state
        self.current_intent = None
        self.executing = False
        
        # Create subscribers
        self.intent_sub = self.create_subscription(
            String,
            'vlm_intent',
            self.intent_callback,
            10
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.gesture_pub = self.create_publisher(String, 'gesture_command', 10)
        
        # Action execution timer
        self.action_timer = None
        
        self.get_logger().info('Intent Executor initialized')
    
    def intent_callback(self, msg: String):
        """Handle incoming intent messages"""
        intent = msg.data
        self.get_logger().info(f'Received intent: {intent}')
        
        # Cancel any ongoing action
        if self.action_timer is not None:
            self.action_timer.cancel()
        
        # Execute the intent
        self.execute_intent(intent)
    
    def execute_intent(self, intent: str):
        """Execute a specific intent"""
        self.current_intent = intent
        self.executing = True
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Executing: {intent}'
        self.status_pub.publish(status_msg)
        
        # Movement intents
        if intent == 'move_forward':
            self.move_forward()
        elif intent == 'move_backward':
            self.move_backward()
        elif intent == 'turn_left':
            self.turn_left()
        elif intent == 'turn_right':
            self.turn_right()
        elif intent == 'stop':
            self.stop_movement()
        
        # Gesture intents
        elif intent == 'wave':
            self.perform_gesture('wave')
        elif intent == 'nod':
            self.perform_gesture('nod')
        
        # Complex behaviors
        elif intent == 'follow':
            self.start_follow_mode()
        elif intent == 'pick_up':
            self.perform_pickup()
        
        else:
            self.get_logger().warn(f'Unknown intent: {intent}')
            self.executing = False
    
    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Stop after duration
        self.action_timer = self.create_timer(
            self.action_duration,
            self.stop_movement
        )
    
    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -self.linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.action_timer = self.create_timer(
            self.action_duration,
            self.stop_movement
        )
    
    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)
        
        self.action_timer = self.create_timer(
            self.action_duration,
            self.stop_movement
        )
    
    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(twist)
        
        self.action_timer = self.create_timer(
            self.action_duration,
            self.stop_movement
        )
    
    def stop_movement(self):
        """Stop all robot movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        if self.action_timer is not None:
            self.action_timer.cancel()
            self.action_timer = None
        
        self.executing = False
        self.get_logger().info('Movement stopped')
    
    def perform_gesture(self, gesture_name: str):
        """Trigger a gesture animation"""
        gesture_msg = String()
        gesture_msg.data = gesture_name
        self.gesture_pub.publish(gesture_msg)
        
        self.get_logger().info(f'Performing gesture: {gesture_name}')
        self.executing = False
    
    def start_follow_mode(self):
        """Start person following mode"""
        # This would integrate with a person detection/tracking system
        self.get_logger().info('Follow mode activated')
        
        status_msg = String()
        status_msg.data = 'follow_mode_active'
        self.status_pub.publish(status_msg)
        
        # TODO: Implement person tracking and following
    
    def perform_pickup(self):
        """Perform object pickup sequence"""
        self.get_logger().info('Performing pickup sequence')
        
        # This would coordinate arm movements and gripper control
        # For now, just publish a gesture command
        gesture_msg = String()
        gesture_msg.data = 'pickup_sequence'
        self.gesture_pub.publish(gesture_msg)
        
        self.executing = False


def main(args=None):
    rclpy.init(args=args)
    node = IntentExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()