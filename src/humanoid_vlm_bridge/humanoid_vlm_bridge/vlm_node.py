#!/usr/bin/env python3
"""
VLM Node - Qwen2.5-Omni Integration for Humanoid Robot

This node handles communication with the Qwen2.5-Omni VLM model running on the gaming PC.
It processes vision and audio inputs, sends them to the VLM, and publishes responses.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64
import json
import threading
import queue
from typing import Optional, Dict, Any

try:
    import torch
    from transformers import AutoModelForCausalLM, AutoProcessor
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class VLMNode(Node):
    """ROS2 node for Qwen2.5-Omni VLM integration"""
    
    def __init__(self):
        super().__init__('vlm_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '~/models/Qwen2.5-Omni-3B')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('process_rate', 2.0)  # Hz, how often to process images
        self.declare_parameter('max_queue_size', 5)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        camera_topic = self.get_parameter('camera_topic').value
        use_compressed = self.get_parameter('use_compressed').value
        process_rate = self.get_parameter('process_rate').value
        max_queue_size = self.get_parameter('max_queue_size').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Image queue for processing
        self.image_queue = queue.Queue(maxsize=max_queue_size)
        self.latest_image = None
        
        # VLM model and processor
        self.model = None
        self.processor = None
        
        # Load model
        if TORCH_AVAILABLE:
            self.load_model(model_path)
        else:
            self.get_logger().error('PyTorch or transformers not available! Install with: pip install torch transformers')
        
        # Create subscribers
        if use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                camera_topic + '/compressed',
                self.compressed_image_callback,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                10
            )
        
        self.text_input_sub = self.create_subscription(
            String,
            'vlm_text_input',
            self.text_input_callback,
            10
        )
        
        self.audio_input_sub = self.create_subscription(
            String,
            'vlm_audio_input',
            self.audio_input_callback,
            10
        )
        
        # Create publishers
        self.response_pub = self.create_publisher(String, 'vlm_response', 10)
        self.intent_pub = self.create_publisher(String, 'vlm_intent', 10)
        self.status_pub = self.create_publisher(Bool, 'vlm_ready', 10)
        
        # Processing thread
        self.processing = True
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()
        
        # Timer for periodic status updates
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('VLM Node initialized')
    
    def load_model(self, model_path: str):
        """Load Qwen2.5-Omni model"""
        try:
            self.get_logger().info(f'Loading Qwen2.5-Omni model from {model_path}...')
            
            # Load processor and model
            self.processor = AutoProcessor.from_pretrained(
                model_path,
                trust_remote_code=True
            )
            
            self.model = AutoModelForCausalLM.from_pretrained(
                model_path,
                device_map=self.device,
                torch_dtype=torch.float16,
                trust_remote_code=True
            )
            
            self.get_logger().info('Model loaded successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None
            self.processor = None
    
    def image_callback(self, msg: Image):
        """Handle raw image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_image = cv_image
            
            # Add to queue if not full
            if not self.image_queue.full():
                self.image_queue.put(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def compressed_image_callback(self, msg: CompressedImage):
        """Handle compressed image messages"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.latest_image = cv_image
            
            if not self.image_queue.full():
                self.image_queue.put(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
    def text_input_callback(self, msg: String):
        """Handle text input for VLM query"""
        if self.latest_image is not None:
            self.process_vlm_query(msg.data, self.latest_image)
        else:
            self.get_logger().warn('No image available for VLM query')
    
    def audio_input_callback(self, msg: String):
        """Handle audio input (transcribed text)"""
        # For Qwen2.5-Omni, audio can be processed as text
        if self.latest_image is not None:
            self.process_vlm_query(msg.data, self.latest_image, is_audio=True)
        else:
            self.get_logger().warn('No image available for audio query')
    
    def process_vlm_query(self, text: str, image: np.ndarray, is_audio: bool = False):
        """Process a VLM query with vision and text"""
        if self.model is None or self.processor is None:
            self.get_logger().error('Model not loaded!')
            return
        
        try:
            # Prepare inputs
            inputs = self.processor(
                text=text,
                images=image,
                return_tensors="pt"
            ).to(self.device)
            
            # Generate response
            with torch.no_grad():
                outputs = self.model.generate(
                    **inputs,
                    max_new_tokens=256,
                    do_sample=True,
                    temperature=0.7,
                    top_p=0.9
                )
            
            # Decode response
            response = self.processor.decode(outputs[0], skip_special_tokens=True)
            
            # Extract intent from response (simple keyword matching)
            intent = self.extract_intent(response)
            
            # Publish response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)
            
            # Publish intent
            if intent:
                intent_msg = String()
                intent_msg.data = intent
                self.intent_pub.publish(intent_msg)
            
            self.get_logger().info(f'VLM Response: {response[:100]}...')
            
        except Exception as e:
            self.get_logger().error(f'Error processing VLM query: {e}')
    
    def extract_intent(self, response: str) -> Optional[str]:
        """Extract action intent from VLM response"""
        response_lower = response.lower()
        
        # Simple keyword-based intent extraction
        intents = {
            'move_forward': ['move forward', 'go forward', 'advance'],
            'move_backward': ['move backward', 'go back', 'reverse'],
            'turn_left': ['turn left', 'rotate left'],
            'turn_right': ['turn right', 'rotate right'],
            'stop': ['stop', 'halt', 'freeze'],
            'follow': ['follow', 'chase', 'track'],
            'wave': ['wave', 'greet', 'hello'],
            'pick_up': ['pick up', 'grab', 'grasp'],
        }
        
        for intent, keywords in intents.items():
            if any(keyword in response_lower for keyword in keywords):
                return intent
        
        return None
    
    def process_loop(self):
        """Background processing loop"""
        while self.processing and rclpy.ok():
            try:
                # This can be used for periodic processing or background tasks
                threading.Event().wait(0.5)
            except Exception as e:
                self.get_logger().error(f'Error in process loop: {e}')
    
    def publish_status(self):
        """Publish VLM ready status"""
        status_msg = Bool()
        status_msg.data = (self.model is not None)
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
        self.processing = False
        if hasattr(self, 'process_thread'):
            self.process_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VLMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()