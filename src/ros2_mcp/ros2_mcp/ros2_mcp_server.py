#!/usr/bin/env python
"""
ROS2 Model Context Protocol Server using FastMCP with SSE support.

This server provides tools to introspect and interact with a ROS2 robotics system.
"""

import json
import logging
import sys
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional, AsyncIterator
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rosidl_runtime_py
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from mcp.server.fastmcp import FastMCP, Context, Image as MCPImage


# Configure logging to stderr
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    stream=sys.stderr
)
logger = logging.getLogger(__name__)


@dataclass
class ROS2Context:
    """Context containing ROS2 node and bridge."""
    node: Node
    cv_bridge: CvBridge


@asynccontextmanager
async def ros2_lifespan(server: FastMCP) -> AsyncIterator[ROS2Context]:
    """Manage ROS2 node lifecycle."""
    logger.info("Initializing ROS2 context...")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 node
    node = Node('mcp_ros2_server')
    cv_bridge = CvBridge()
    
    logger.info("ROS2 node created successfully")
    
    try:
        yield ROS2Context(node=node, cv_bridge=cv_bridge)
    finally:
        # Cleanup
        logger.info("Shutting down ROS2 context...")
        node.destroy_node()
        rclpy.shutdown()
        logger.info("ROS2 context shutdown complete")


# Create MCP server with ROS2 lifespan management
mcp = FastMCP(
    "ROS2 MCP Server",
    dependencies=["rclpy", "cv_bridge", "opencv-python"],
    lifespan=ros2_lifespan
)


def serialize_ros_message(msg: Any, omit_arrays: bool = True) -> Dict[str, Any]:
    """
    Convert ROS message to JSON-serializable dict.
    
    Args:
        msg: ROS message instance
        omit_arrays: If True, omit array fields to keep response small
        
    Returns:
        Dictionary representation of the message
    """
    try:
        # Convert to ordered dict first
        msg_dict = message_to_ordereddict(msg)
        
        # If omitting arrays, remove array fields
        if omit_arrays:
            return _remove_arrays(msg_dict)
        
        return dict(msg_dict)
    except Exception as e:
        logger.warning(f"Failed to convert message to dict: {e}")
        # Fallback: manually extract fields
        result = {}
        for field in msg.get_fields_and_field_types():
            try:
                value = getattr(msg, field)
                if omit_arrays and hasattr(value, '__len__') and not isinstance(value, str):
                    continue
                result[field] = value
            except Exception as field_error:
                logger.warning(f"Failed to extract field {field}: {field_error}")
                continue
        return result


def _remove_arrays(data: Any) -> Any:
    """Recursively remove array fields from data structure."""
    if isinstance(data, dict):
        result = {}
        for key, value in data.items():
            if isinstance(value, list) and len(value) > 0:
                # Skip arrays, but include empty lists as they're often metadata
                continue
            elif isinstance(value, dict):
                result[key] = _remove_arrays(value)
            else:
                result[key] = value
        return result
    elif isinstance(data, list):
        return [_remove_arrays(item) for item in data]
    else:
        return data


def _wait_for_messages(
    node: Node,
    subscription: Any,
    message_count: int,
    timeout_sec: float = 2.0
) -> bool:
    """
    Wait for messages with proper timeout handling.
    
    Returns:
        True if expected messages received, False if timeout
    """
    rate = node.create_rate(20)  # 20 Hz for responsive checking
    elapsed = 0.0
    dt = 0.05  # 50ms intervals
    
    # Get the callback's received count - this assumes the callback
    # updates some external counter that we can check
    start_time = node.get_clock().now()
    
    while elapsed < timeout_sec:
        rclpy.spin_once(node, timeout_sec=dt)
        elapsed += dt
        rate.sleep()
    
    return False  # We'll handle this differently in each tool


@mcp.tool()
def topic_echo(
    ctx: Context,
    topic_name: str,
    message_count: int = 1,
    include_arrays: bool = False,
    timeout_sec: float = 2.0
) -> Dict[str, Any]:
    """
    Receive a fixed number of messages from a ROS2 topic.
    
    Args:
        topic_name: Name of the ROS2 topic to echo
        message_count: Number of messages to receive (default: 1)
        include_arrays: Whether to include array data fields (default: False)
        timeout_sec: Timeout in seconds (default: 2.0)
        
    Returns:
        Dictionary containing the received messages
    """
    logger.info(f"Echoing topic: {topic_name}, count: {message_count}, timeout: {timeout_sec}s")
    
    ros2_ctx = ctx.request_context.lifespan_context
    node = ros2_ctx.node
    
    # Get topic info
    topic_list = node.get_topic_names_and_types()
    topic_info = next((t for t in topic_list if t[0] == topic_name), None)
    
    if not topic_info:
        error_msg = f"Topic '{topic_name}' not found. Available topics: {[t[0] for t in topic_list[:10]]}"
        logger.error(error_msg)
        return {"error": error_msg}
    
    topic_type = topic_info[1][0]  # Get first message type
    logger.info(f"Topic type: {topic_type}")
    
    try:
        # Get message class
        msg_class = get_message(topic_type)
    except Exception as e:
        error_msg = f"Failed to get message class for type '{topic_type}': {str(e)}"
        logger.error(error_msg)
        return {"error": error_msg}
    
    # Message collection
    messages = []
    received_count = 0
    
    def message_callback(msg):
        nonlocal received_count
        if received_count < message_count:
            try:
                serialized = serialize_ros_message(msg, omit_arrays=not include_arrays)
                messages.append({
                    "timestamp": node.get_clock().now().to_msg(),
                    "data": serialized
                })
                received_count += 1
                logger.debug(f"Received message {received_count}/{message_count}")
            except Exception as e:
                logger.error(f"Failed to serialize message: {e}")
    
    # Create subscription
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=message_count
    )
    
    subscription = node.create_subscription(
        msg_class,
        topic_name,
        message_callback,
        qos
    )
    
    # Wait for messages with timeout
    rate = node.create_rate(20)  # 20 Hz
    elapsed = 0.0
    dt = 0.05  # 50ms intervals
    
    while received_count < message_count and elapsed < timeout_sec:
        rclpy.spin_once(node, timeout_sec=dt)
        elapsed += dt
        rate.sleep()
    
    # Cleanup
    node.destroy_subscription(subscription)
    
    # Check if we got the expected number of messages
    if received_count == 0:
        error_msg = f"No messages received from topic '{topic_name}' within {timeout_sec}s timeout. Check if the topic is publishing data."
        logger.error(error_msg)
        return {"error": error_msg}
    
    if received_count < message_count:
        warning_msg = f"Only received {received_count}/{message_count} messages from '{topic_name}' within {timeout_sec}s timeout"
        logger.warning(warning_msg)
        # Return partial results with warning
        return {
            "topic": topic_name,
            "type": topic_type,
            "message_count": len(messages),
            "messages": messages,
            "warning": warning_msg
        }
    
    logger.info(f"Successfully received {len(messages)} messages from {topic_name}")
    return {
        "topic": topic_name,
        "type": topic_type,
        "message_count": len(messages),
        "messages": messages
    }


@mcp.tool()
def get_image(
    ctx: Context,
    topic_name: str,
    timeout_sec: float = 2.0
) -> MCPImage:
    """
    Get a single image from a ROS2 Image topic and return as MCP Image for VLM usage.
    
    Args:
        topic_name: Name of the ROS2 Image topic
        timeout_sec: Timeout in seconds (default: 2.0)
        
    Returns:
        MCP Image object that VLMs can directly use
    """
    logger.info(f"Getting image from topic: {topic_name}, timeout: {timeout_sec}s")
    
    ros2_ctx = ctx.request_context.lifespan_context
    node = ros2_ctx.node
    cv_bridge = ros2_ctx.cv_bridge
    
    # Get topic info
    topic_list = node.get_topic_names_and_types()
    topic_info = next((t for t in topic_list if t[0] == topic_name), None)
    
    if not topic_info:
        available_image_topics = [
            t[0] for t in topic_list 
            if any('Image' in msg_type for msg_type in t[1])
        ]
        error_msg = f"Topic '{topic_name}' not found. Available Image topics: {available_image_topics}"
        logger.error(error_msg)
        raise ValueError(error_msg)
    
    topic_type = topic_info[1][0]
    
    # Verify this is an Image topic
    if topic_type != 'sensor_msgs/msg/Image':
        error_msg = f"Topic '{topic_name}' is not an Image topic (type: {topic_type}). Use topic_echo for non-image topics."
        logger.error(error_msg)
        raise ValueError(error_msg)
    
    logger.info(f"Confirmed Image topic type: {topic_type}")
    
    # Image collection
    received_image = None
    image_received = False
    
    def image_callback(msg):
        nonlocal received_image, image_received
        if not image_received:
            try:
                # Convert ROS Image to OpenCV format
                if msg.encoding == 'rgb8':
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                elif msg.encoding == 'bgr8':
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    # Convert BGR to RGB for consistency
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else:
                    # Try to convert to RGB8 for other encodings
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                
                received_image = cv_image
                image_received = True
                logger.debug(f"Successfully converted image: {msg.width}x{msg.height}, encoding: {msg.encoding}")
                
            except Exception as e:
                logger.error(f"Failed to process image: {e}")
                raise ValueError(f"Failed to process image: {str(e)}")
    
    # Create subscription
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )
    
    subscription = node.create_subscription(
        Image,
        topic_name,
        image_callback,
        qos
    )
    
    # Wait for image with timeout
    rate = node.create_rate(20)  # 20 Hz
    elapsed = 0.0
    dt = 0.05  # 50ms intervals
    
    while not image_received and elapsed < timeout_sec:
        rclpy.spin_once(node, timeout_sec=dt)
        elapsed += dt
        rate.sleep()
    
    # Cleanup
    node.destroy_subscription(subscription)
    
    # Check if we got an image
    if not image_received or received_image is None:
        error_msg = f"No image received from topic '{topic_name}' within {timeout_sec}s timeout. Check if the topic is publishing images."
        logger.error(error_msg)
        raise ValueError(error_msg)
    
    # Convert to PNG bytes for MCP Image
    try:
        # Convert RGB to BGR for OpenCV encoding
        cv_image_bgr = cv2.cvtColor(received_image, cv2.COLOR_RGB2BGR)
        success, buffer = cv2.imencode('.png', cv_image_bgr)
        
        if not success:
            error_msg = "Failed to encode image as PNG"
            logger.error(error_msg)
            raise ValueError(error_msg)
        
        image_bytes = buffer.tobytes()
        
        logger.info(f"Successfully retrieved and encoded image from {topic_name}")
        return MCPImage(data=image_bytes, media_type="image/png")
        
    except Exception as e:
        error_msg = f"Failed to encode image: {str(e)}"
        logger.error(error_msg)
        raise ValueError(error_msg)


@mcp.tool()
def list_topics(ctx: Context) -> Dict[str, Any]:
    """
    List all available ROS2 topics with their message types.
    
    Returns:
        Dictionary containing all topics and their types
    """
    logger.info("Listing ROS2 topics")
    
    ros2_ctx = ctx.request_context.lifespan_context
    node = ros2_ctx.node
    
    try:
        # Get all topics
        topic_list = node.get_topic_names_and_types()
        
        topics = []
        image_topics = []
        
        for topic_name, topic_types in topic_list:
            topic_entry = {
                "name": topic_name,
                "types": topic_types
            }
            topics.append(topic_entry)
            
            # Track image topics separately for convenience
            if any('Image' in msg_type for msg_type in topic_types):
                image_topics.append(topic_name)
        
        logger.info(f"Found {len(topics)} topics, {len(image_topics)} image topics")
        return {
            "topics": topics,
            "count": len(topics),
            "image_topics": image_topics
        }
    
    except Exception as e:
        error_msg = f"Failed to list topics: {str(e)}"
        logger.error(error_msg)
        return {"error": error_msg}


@mcp.tool()
def introspect_interface(interface_type: str) -> Dict[str, Any]:
    """
    Provide details about a ROS2 interface type (message, service, or action).
    
    Args:
        interface_type: The interface type (e.g., 'sensor_msgs/msg/Image')
        
    Returns:
        Dictionary containing interface details
    """
    logger.info(f"Introspecting interface: {interface_type}")
    
    try:
        # Try to get the message class
        msg_class = get_message(interface_type)
        
        # Get field information
        fields = msg_class.get_fields_and_field_types()
        
        # Build field details
        field_details = []
        for field_name, field_type in fields.items():
            field_details.append({
                "name": field_name,
                "type": field_type
            })
        
        # Get constants if any
        constants = {}
        if hasattr(msg_class, '__constants__'):
            constants = msg_class.__constants__
        
        logger.info(f"Successfully introspected {interface_type}")
        return {
            "interface_type": interface_type,
            "fields": field_details,
            "constants": constants,
            "full_name": msg_class.__name__,
            "module": msg_class.__module__
        }
    
    except Exception as e:
        error_msg = f"Failed to introspect interface '{interface_type}': {str(e)}"
        logger.error(error_msg)
        return {"error": error_msg}


def main():
    """Main entry point for the ROS2 MCP Server."""
    logger.info("Starting ROS2 MCP Server with SSE support...")
    
    # Run the server - this will start the SSE server
    # The SSE endpoint will be available at the server's configured path
    try:
        mcp.run()
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
        raise


if __name__ == "__main__":
    main()