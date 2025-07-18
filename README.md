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

### using in your ROS2 workspace

To use this module in your ROS2 workspace, follow these steps:

1. **Clone the repository** into your ROS2 workspace's `src` directory:
    ```bash
    cd ~/your_ros2_workspace/src
    git clone <repository-url>
    ```

2. **Install dependencies** using rosdep to automatically handle both ROS and pip dependencies:
    ```bash
    cd ~/your_ros2_workspace
    rosdep install -i --from-paths src/
    ```
    This will install the required `mcp[cli]` pip dependency along with any other ROS dependencies.

3. **Build the workspace**:
    ```bash
    colcon build
    ```

4. **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

5. **Run the nodes** as needed:
    ```bash
    ros2 run ros2_mcp <node_name>
    ```

**Note:** Make sure you have Python 3 and pip installed on your system, as the MCP CLI tools require Python dependencies that will be installed automatically through rosdep.

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

## Using the SSE Endpoint with an LLM

Once running, the MCP server's SSE interface will be available at:
- **Local**: `http://localhost:8000/sse`
- **Health check**: `http://localhost:8000/health`

### Expose to e.g. Claude

To expose the MCP server to external AI assistants like Claude, the simplest is to use `mcp-proxy` to expose the SSE endpoint directly as an STDIO MCP server (see https://github.com/sparfenyuk/mcp-proxy?tab=readme-ov-file#12-example-usage):


Example for Claude Desktop:

```json
{
  "mcpServers": {
    "mcp-proxy": {
      "command": "uv",
      "args": [
        "tool",
        "run",
        "mcp-proxy",
        "http://localhost:8000/sse"
      ]
    }
  }
}
```
