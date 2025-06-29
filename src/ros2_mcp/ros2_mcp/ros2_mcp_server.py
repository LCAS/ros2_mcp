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
from mcp import types


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
    
    # Check if ROS2 is already initialized
    if not rclpy.ok():
        logger.info("Initializing ROS2...")
        rclpy.init()
    else:
        logger.info("ROS2 already initialized, reusing existing context")
    
    # Create ROS2 node
    node = Node('mcp_ros2_server')
    cv_bridge = CvBridge()
    
    logger.info("ROS2 node created successfully")
    
    try:
        yield ROS2Context(node=node, cv_bridge=cv_bridge)
    finally:
        # Cleanup
        logger.info("Shutting down ROS2 context...")
        try:
            node.destroy_node()
            logger.info("ROS2 node destroyed")
        except Exception as e:
            logger.warning(f"Error destroying node: {e}")
        
        # Only shutdown if we're still in a valid context
        try:
            if rclpy.ok():
                rclpy.shutdown()
                logger.info("ROS2 context shutdown complete")
            else:
                logger.info("ROS2 context already shutdown")
        except Exception as e:
            logger.warning(f"Error during ROS2 shutdown: {e}")


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
        
        # Convert any non-serializable types
        msg_dict = _convert_ros_types(msg_dict)
        
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
                # Convert ROS types to serializable formats
                value = _convert_ros_value(value)
                if omit_arrays and hasattr(value, '__len__') and not isinstance(value, str):
                    continue
                result[field] = value
            except Exception as field_error:
                logger.warning(f"Failed to extract field {field}: {field_error}")
                continue
        return result


def _convert_ros_types(data: Any) -> Any:
    """Recursively convert ROS2 types to JSON-serializable types."""
    if isinstance(data, dict):
        result = {}
        for key, value in data.items():
            result[key] = _convert_ros_types(value)
        return result
    elif isinstance(data, list):
        return [_convert_ros_types(item) for item in data]
    else:
        return _convert_ros_value(data)


def _convert_ros_value(value: Any) -> Any:
    """Convert a single ROS value to a serializable type."""
    # Handle ROS2 Time objects
    if hasattr(value, 'sec') and hasattr(value, 'nanosec'):
        return {
            "sec": value.sec,
            "nanosec": value.nanosec,
            "timestamp": value.sec + value.nanosec / 1e9
        }
    
    # Handle ROS2 Duration objects
    elif hasattr(value, 'sec') and hasattr(value, 'nanosec') and 'Duration' in str(type(value)):
        return {
            "sec": value.sec,
            "nanosec": value.nanosec,
            "duration": value.sec + value.nanosec / 1e9
        }
    
    # Handle numpy arrays
    elif hasattr(value, 'tolist'):
        try:
            return value.tolist()
        except:
            return str(value)
    
    # Handle bytes
    elif isinstance(value, bytes):
        try:
            return value.decode('utf-8')
        except:
            return list(value)  # Convert to list of integers
    
    # For other complex objects, try to get their string representation
    elif hasattr(value, '__dict__') and not isinstance(value, (str, int, float, bool)):
        try:
            return str(value)
        except:
            return f"<{type(value).__name__}>"
    
    return value


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
                # Convert ROS2 time to serializable format
                ros_time = node.get_clock().now()
                timestamp_sec = ros_time.nanoseconds / 1e9  # Convert to seconds as float
                messages.append({
                    "timestamp": timestamp_sec,
                    "timestamp_iso": f"{timestamp_sec:.9f}",  # Human readable format
                    "data": serialized
                })
                received_count += 1
                logger.info(f"Received message {received_count}/{message_count} from {topic_name}")
            except Exception as e:
                logger.error(f"Failed to serialize message: {e}")
    
    # Create subscription with more reliable QoS
    # Try RELIABLE first, fallback to BEST_EFFORT if needed
    qos_profiles = [
        QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, message_count)
        ),
        QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, message_count)
        )
    ]
    
    subscription = None
    for qos in qos_profiles:
        try:
            subscription = node.create_subscription(
                msg_class,
                topic_name,
                message_callback,
                qos
            )
            logger.info(f"Created subscription to {topic_name} with {qos.reliability.name} QoS")
            break
        except Exception as e:
            logger.warning(f"Failed to create subscription with {qos.reliability.name} QoS: {e}")
            continue
    
    if subscription is None:
        error_msg = f"Failed to create subscription to topic '{topic_name}'"
        logger.error(error_msg)
        return {"error": error_msg}
    
    logger.info(f"Waiting for {message_count} messages from {topic_name}...")
    
    # Wait for messages with timeout
    import time
    start_time = time.time()
    
    while received_count < message_count and (time.time() - start_time) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)  # Small sleep to prevent busy waiting
        
        # Log progress every second
        if int(time.time() - start_time) % 1 == 0 and int(time.time() - start_time) > 0:
            logger.info(f"Waiting for messages... received {received_count}/{message_count}, elapsed: {time.time() - start_time:.1f}s")
    
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
    import time
    start_time = time.time()
    
    while not image_received and (time.time() - start_time) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)  # Small sleep to prevent busy waiting
    
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
        # Create MCP Image with just the data parameter
        return MCPImage(data=image_bytes)
        
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


@mcp.resource("ros2://system-status")
def get_ros2_system_status() -> str:
    """
    Resource providing live ROS2 system status information.
    
    Returns:
        JSON string containing current ROS2 system status
    """
    logger.info("Generating ROS2 system status resource")
    
    # Get the ROS2 context from the server's lifespan context
    # This requires accessing the FastMCP server instance
    # For now, we'll use a simpler approach that doesn't require ctx
    try:
        # Check if ROS2 is initialized
        if not rclpy.ok():
            return json.dumps({
                "timestamp": 0,
                "error": "ROS2 not initialized",
                "system_health": {
                    "rclpy_ok": False,
                    "node_active": False,
                    "discovery_working": False
                }
            }, indent=2)
        
        # Create a temporary node for status gathering
        temp_node = Node('mcp_status_node')
        node = temp_node
    
        # Get topic information
        topic_list = node.get_topic_names_and_types()
        topics_by_type = {}
        image_topics = []
        
        for topic_name, topic_types in topic_list:
            for topic_type in topic_types:
                if topic_type not in topics_by_type:
                    topics_by_type[topic_type] = []
                topics_by_type[topic_type].append(topic_name)
                
                # Track image topics
                if 'Image' in topic_type:
                    image_topics.append(topic_name)
        
        # Get node names (requires ROS2 discovery)
        try:
            node_names = node.get_node_names()
        except Exception as e:
            logger.warning(f"Could not get node names: {e}")
            node_names = ["Unable to retrieve node names"]
        
        # Get parameter information for our own node
        try:
            param_names = node.list_parameters([], 10).names
        except Exception as e:
            logger.warning(f"Could not get parameters: {e}")
            param_names = []
        
        # Generate status report
        status = {
            "timestamp": node.get_clock().now().nanoseconds / 1e9,
            "node_info": {
                "mcp_node_name": node.get_name(),
                "mcp_node_namespace": node.get_namespace(),
                "discovered_nodes": node_names,
                "node_count": len(node_names) if isinstance(node_names, list) else 0
            },
            "topics": {
                "total_count": len(topic_list),
                "by_type": topics_by_type,
                "image_topics": image_topics,
                "image_topic_count": len(image_topics)
            },
            "parameters": {
                "available_params": param_names,
                "param_count": len(param_names)
            },
            "system_health": {
                "rclpy_ok": rclpy.ok(),
                "node_active": True,
                "discovery_working": len(node_names) > 0 if isinstance(node_names, list) else False
            }
        }
        
        logger.info(f"System status generated: {len(topic_list)} topics, {len(node_names) if isinstance(node_names, list) else 0} nodes")
        
        # Cleanup temporary node
        temp_node.destroy_node()
        
        return json.dumps(status, indent=2)
        
    except Exception as e:
        error_status = {
            "timestamp": 0,
            "error": f"Failed to generate system status: {str(e)}",
            "system_health": {
                "rclpy_ok": rclpy.ok(),
                "node_active": False,
                "discovery_working": False
            }
        }
        logger.error(f"Error generating system status: {e}")
        
        # Try to cleanup temporary node if it exists
        try:
            if 'temp_node' in locals():
                temp_node.destroy_node()
        except:
            pass
            
        return json.dumps(error_status, indent=2)


@mcp.prompt()
def analyze_robot_state() -> types.Prompt:
    """
    Prompt for analyzing the current state of the robot system.
    
    This prompt helps LLMs understand and analyze the current ROS2 system state,
    including active topics, available sensors, and system health.
    """
    return types.Prompt(
        name="analyze_robot_state",
        description="Analyze the current state of the ROS2 robot system",
        arguments=[
            types.PromptArgument(
                name="focus_area",
                description="Specific area to focus analysis on (e.g., 'sensors', 'navigation', 'vision', 'all')",
                required=False
            )
        ],
        messages=[
            types.PromptMessage(
                role="user",
                content=types.TextContent(
                    type="text",
                    text="""Analyze the current ROS2 robot system state. Please:

1. First, get the system status using the ros2://system-status resource
2. List all available topics to understand what sensors and systems are active
3. Based on the focus area (if specified), provide a detailed analysis of:
   - System health and connectivity
   - Available sensors and their types
   - Active topics and their publication status
   - Any potential issues or recommendations

If a focus area is specified, concentrate on that particular aspect:
- 'sensors': Focus on sensor topics, image streams, and sensor health
- 'navigation': Look for navigation-related topics and localization data
- 'vision': Analyze camera feeds and computer vision topics
- 'all': Provide comprehensive system analysis

Please provide actionable insights and suggest specific tools or topics to investigate further if needed."""
                )
            )
        ]
    )


@mcp.prompt()
def debug_topic_issues() -> types.Prompt:
    """
    Prompt for debugging ROS2 topic communication issues.
    
    This prompt guides LLMs through systematic topic debugging procedures.
    """
    return types.Prompt(
        name="debug_topic_issues",
        description="Debug ROS2 topic communication and data flow issues",
        arguments=[
            types.PromptArgument(
                name="topic_name",
                description="Name of the specific topic to debug",
                required=True
            ),
            types.PromptArgument(
                name="expected_behavior",
                description="Description of what the topic should be doing",
                required=False
            )
        ],
        messages=[
            types.PromptMessage(
                role="user",
                content=types.TextContent(
                    type="text",
                    text="""Debug the ROS2 topic: {topic_name}

Expected behavior: {expected_behavior}

Please perform the following diagnostic steps:

1. First, verify the topic exists by listing all available topics
2. Check the topic's message type and structure using introspect_interface
3. Attempt to echo messages from the topic with different timeout values
4. Analyze the message content and frequency
5. If it's an image topic, try to retrieve and analyze image data

Based on your findings, diagnose potential issues such as:
- Topic not publishing (no data received)
- Incorrect message format or encoding
- Network/timing issues
- Publisher/subscriber compatibility problems

Provide specific recommendations for resolving any identified issues."""
                )
            )
        ]
    )


@mcp.prompt()
def sensor_data_collection() -> types.Prompt:
    """
    Prompt for systematic sensor data collection and analysis.
    
    This prompt helps LLMs efficiently collect and analyze sensor data from the robot.
    """
    return types.Prompt(
        name="sensor_data_collection",
        description="Collect and analyze sensor data from the ROS2 system",
        arguments=[
            types.PromptArgument(
                name="sensor_types",
                description="Comma-separated list of sensor types to focus on (e.g., 'camera,lidar,imu')",
                required=False
            ),
            types.PromptArgument(
                name="duration_seconds",
                description="How long to collect data for each sensor (default: 2)",
                required=False
            )
        ],
        messages=[
            types.PromptMessage(
                role="user",
                content=types.TextContent(
                    type="text",
                    text="""Collect sensor data from the ROS2 system.

Target sensor types: {sensor_types}
Collection duration: {duration_seconds} seconds per sensor

Please follow this systematic approach:

1. Get the current system status to identify available sensors
2. List all topics and identify sensor-related topics
3. For each relevant sensor type:
   - Camera/Image sensors: Use get_image tool to capture current images
   - Other sensors: Use topic_echo to collect recent data samples
4. Analyze the collected data for:
   - Data quality and completeness
   - Sensor calibration status
   - Timestamp consistency
   - Any anomalies or error conditions

If no specific sensor types are provided, collect data from all available sensors.

Provide a summary of:
- What sensors are active and responding
- Quality assessment of each sensor's data
- Any detected issues or recommendations
- Suggestions for further investigation if needed"""
                )
            )
        ]
    )


@mcp.prompt()
def robot_status_report() -> types.Prompt:
    """
    Prompt for generating comprehensive robot status reports.
    
    This prompt creates detailed reports suitable for operators or maintenance teams.
    """
    return types.Prompt(
        name="robot_status_report",
        description="Generate a comprehensive robot status report",
        arguments=[
            types.PromptArgument(
                name="report_type",
                description="Type of report: 'operational', 'maintenance', 'diagnostic'",
                required=False
            )
        ],
        messages=[
            types.PromptMessage(
                role="user",
                content=types.TextContent(
                    type="text",
                    text="""Generate a comprehensive robot status report.

Report type: {report_type}

Please create a structured report including:

**System Overview:**
1. Get the ROS2 system status resource
2. List all active topics and nodes
3. Assess overall system health

**Sensor Status:**
4. Identify all sensor topics
5. Test key sensors for responsiveness
6. Report on data quality and availability

**Communication Health:**
7. Check topic publication rates
8. Identify any communication issues
9. Verify critical data flows

**Recommendations:**
10. Highlight any issues requiring attention
11. Suggest maintenance actions if needed
12. Recommend monitoring priorities

Format the report based on the report type:
- 'operational': Focus on current performance and immediate issues
- 'maintenance': Emphasize system health and preventive actions
- 'diagnostic': Detailed technical analysis for troubleshooting

Make the report actionable and include specific topic names, error details, and next steps where applicable."""
                )
            )
        ]
    )


def main():
    """Main entry point for the ROS2 MCP Server."""
    logger.info("Starting ROS2 MCP Server with SSE support...")
    
    # Run the server - this will start the SSE server
    # The SSE endpoint will be available at the server's configured path
    try:
        mcp.run(transport="sse")
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
        # Ensure cleanup on any error
        try:
            if rclpy.ok():
                rclpy.shutdown()
                logger.info("Emergency ROS2 shutdown completed")
        except Exception as cleanup_error:
            logger.warning(f"Error during emergency cleanup: {cleanup_error}")
        raise


if __name__ == "__main__":
    main()