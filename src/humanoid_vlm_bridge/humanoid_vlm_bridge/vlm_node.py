#!/usr/bin/env python3
"""
VLM Node - Qwen2.5-Omni Integration for Humanoid Robot

This node handles communication with the Qwen2.5-Omni VLM model running on the gaming PC.
The VLM receives video and audio from Android tablet directly (not through ROS).
This node only handles:
  - Text queries from Android → VLM
  - VLM responses → Robot actions (intents)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import json
import threading
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
        self.declare_parameter('use_local_model', True)  # If false, expects external VLM API
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        use_local_model = self.get_parameter('use_local_model').value
        
        # VLM model and processor (optional if Android handles it)
        self.model = None
        self.processor = None
        
        # Load model if running locally on this machine
        if use_local_model and TORCH_AVAILABLE:
            self.load_model(model_path)
        else:
            if not TORCH_AVAILABLE:
                self.get_logger().warn('PyTorch not available. Expecting VLM responses from Android.')
            else:
                self.get_logger().info('Local model disabled. Expecting VLM responses from Android.')
        
        # Create subscribers
        # Text input from Android app (user query)
        self.text_input_sub = self.create_subscription(
            String,
            'vlm_text_input',
            self.text_input_callback,
            10
        )
        
        # Direct VLM response from Android (if Android runs the VLM)
        self.vlm_response_sub = self.create_subscription(
            String,
            'vlm_response_from_android',
            self.vlm_response_callback,
            10
        )
        
        # Audio input from Android (transcribed speech)
        self.audio_input_sub = self.create_subscription(
            String,
            'vlm_audio_input',
            self.audio_input_callback,
            10
        )
        
        # Create publishers
        # Send responses back to Android display
        self.response_pub = self.create_publisher(String, 'vlm_response', 10)
        
        # Intent commands for robot actions
        self.intent_pub = self.create_publisher(String, 'vlm_intent', 10)
        
        # Status indicator
        self.status_pub = self.create_publisher(Bool, 'vlm_ready', 10)
        
        # Timer for periodic status updates
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('VLM Node initialized (Android handles camera/video)')
        self.get_logger().info('Listening for text/audio queries from Android')
    
    def load_model(self, model_path: str):
        """Load Qwen2.5-Omni model (optional if VLM runs on Gaming PC)"""
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
            
            self.get_logger().info('Model loaded successfully on Gaming PC!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.get_logger().info('Will expect VLM responses from Android instead')
            self.model = None
            self.processor = None
    
    def text_input_callback(self, msg: String):
        """
        Handle text input from Android app
        Android sends user query → Gaming PC processes with VLM → Returns intent
        """
        query = msg.data
        self.get_logger().info(f'Received text query from Android: "{query}"')
        
        # If VLM is running on Gaming PC
        if self.model is not None and self.processor is not None:
            self.process_text_query(query)
        else:
            # VLM is running on Android, just echo back
            self.get_logger().info('No local VLM. Expecting response from Android via /vlm_response_from_android')
    
    def audio_input_callback(self, msg: String):
        """
        Handle audio input (transcribed speech) from Android
        """
        transcribed_text = msg.data
        self.get_logger().info(f'Received audio input (transcribed): "{transcribed_text}"')
        
        # Process same as text
        if self.model is not None:
            self.process_text_query(transcribed_text)
    
    def vlm_response_callback(self, msg: String):
        """
        Handle VLM response from Android (if Android runs the VLM)
        Android processes camera/audio with VLM → Sends response to ROS
        """
        response = msg.data
        self.get_logger().info(f'Received VLM response from Android: "{response[:100]}..."')
        
        # Extract intent from response
        intent = self.extract_intent(response)
        
        # Publish response back (for logging/display)
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        
        # Publish intent for robot action
        if intent:
            intent_msg = String()
            intent_msg.data = intent
            self.intent_pub.publish(intent_msg)
            self.get_logger().info(f'Extracted intent: {intent}')
    
    def process_text_query(self, text: str):
        """
        Process a text query using local VLM (Gaming PC)
        Note: Without camera images, this is text-only mode
        """
        if self.model is None or self.processor is None:
            self.get_logger().error('Model not loaded!')
            return
        
        try:
            # Prepare text-only input (no image)
            inputs = self.processor(
                text=text,
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
            
            # Extract intent from response
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
        """
        Extract action intent from VLM response
        Simple keyword matching - can be enhanced with better NLP
        """
        response_lower = response.lower()
        
        # Intent keywords mapping
        intents = {
            'move_forward': ['move forward', 'go forward', 'advance', 'go ahead', 'walk forward'],
            'move_backward': ['move backward', 'go back', 'reverse', 'back up', 'move back'],
            'turn_left': ['turn left', 'rotate left', 'go left', 'left turn'],
            'turn_right': ['turn right', 'rotate right', 'go right', 'right turn'],
            'stop': ['stop', 'halt', 'freeze', 'stand still', 'don\'t move'],
            'follow': ['follow', 'chase', 'track', 'come with', 'follow me'],
            'wave': ['wave', 'greet', 'hello', 'hi', 'say hi'],
            'nod': ['nod', 'yes', 'agree', 'nod head'],
            'look_around': ['look around', 'scan area', 'check surroundings'],
            'dance': ['dance', 'move to music', 'groove'],
        }
        
        # Check for intent keywords
        for intent, keywords in intents.items():
            if any(keyword in response_lower for keyword in keywords):
                return intent
        
        return None
    
    def publish_status(self):
        """Publish VLM ready status"""
        status_msg = Bool()
        # Ready if model loaded OR expecting responses from Android
        status_msg.data = (self.model is not None) or True
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
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
