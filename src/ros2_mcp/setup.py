from setuptools import find_packages, setup

package_name = "ros2_mcp"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "mcp[cli]",
        "rclpy",
        "cv_bridge",
        "opencv-python",
        "rosidl_runtime_py",
    ],
    zip_safe=True,
    maintainer="Marc Hanheide",
    maintainer_email="marc@hanheide.net",
    description="An MCP for ROS2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ros2_mcp_server = ros2_mcp.ros2_mcp_server:main"],
    },
)
