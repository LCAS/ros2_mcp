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

## Docker Usage

### Pre-built Images

Docker images are automatically built and published to the L-CAS registry:

```bash
docker pull lcas.lincoln.ac.uk/<repository-name>:latest
```

### Building Locally

To build the Docker image locally:

```bash
docker build -t ros2-mcp .
```

### Running the Container

```bash
# Run with ROS2 network access
docker run -it --rm --network host ros2-mcp

# Run with custom ROS_DOMAIN_ID
docker run -it --rm --network host -e ROS_DOMAIN_ID=42 ros2-mcp
```

### Docker Compose (Recommended)

For easier deployment with the SSE interface exposed on port 8000:

```bash
# Start the MCP server with Docker Compose
docker compose up -d

# View logs
docker compose logs -f ros2-mcp-server

# Stop the service
docker compose down
```

#### Configuration

The Docker Compose setup can be configured via environment variables in the `.env` file:

- `DOCKER_REPO`: Docker repository name (default: `ros2-mcp`)
- `TAG`: Image tag to use (default: `latest`)
- `ROS_DOMAIN_ID`: ROS2 domain ID (default: `0`)
- `MCP_HOST`: Host to bind the MCP server (default: `0.0.0.0`)
- `MCP_PORT`: Port for the SSE interface (default: `8000`)

#### SSE Endpoint

Once running, the MCP server's SSE interface will be available at:
- **Local**: `http://localhost:8000`
- **Health check**: `http://localhost:8000/health`

The Docker image is based on `lcas.lincoln.ac.uk/lcas/ros-docker-images:humble-2` and includes all necessary dependencies for the ROS2 MCP server.

