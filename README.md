# ROS2 MCP Server

A Model Context Protocol (MCP) server that provides AI assistants with tools to introspect and interact with ROS2 robotics systems. This server enables Large Language Models (LLMs) and other AI systems to understand, monitor, and analyze ROS2 robot systems in real-time.

## Overview

The ROS2 MCP Server bridges the gap between AI assistants and robotics systems by exposing ROS2 functionality through the Model Context Protocol. It provides tools for topic monitoring, sensor data collection, system introspection, and real-time robot state analysis.

## Features

### Core Capabilities
- **Topic Monitoring**: Echo messages from any ROS2 topic with configurable message counts and timeouts
- **Image Processing**: Direct image retrieval from ROS2 Image topics for VLM (Vision Language Model) analysis
- **System Introspection**: List all available topics, nodes, and message types
- **Interface Analysis**: Detailed inspection of ROS2 message/service/action interfaces
- **Real-time Status**: Live system health monitoring and status reporting

### AI-Friendly Design
- **Structured Prompts**: Pre-built prompts for common robotics analysis tasks
- **Resource Streaming**: Live system status via MCP resources
- **VLM Integration**: Direct image data provision for computer vision analysis
- **Error Handling**: Robust error reporting and fallback mechanisms

## Installation

### Prerequisites
- ROS2 (Humble, Iron, or Jazzy)
- Python 3.8+
- OpenCV (`cv_bridge` package)

