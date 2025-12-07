# Quickstart Guide: AI-Native Robotics Textbook Development

This guide provides instructions for setting up the necessary development and student environments for the AI-Native Robotics Textbook.

## For Textbook Authors/Contributors (Content Development Environment)

### 1. Docusaurus Setup
To develop content for the textbook, ensure you have Node.js and npm installed.
```bash
# Install Docusaurus CLI globally
npm install -g @docusaurus/cli
# Navigate to the book-content directory
cd book-content/
# Install project dependencies
npm install
# Start the local development server
npm run start
```
This will start a local server and open the textbook in your browser, updating live with changes to Markdown files.

### 2. Markdown Editor
Any Markdown editor will suffice. Consider extensions for Docusaurus-specific features or linting.

### 3. Git Workflow
Content should be managed via Git. Follow standard branching and pull request workflows.

## For Students (Learning Environment)

### 1. Operating System
The recommended operating system for following the practical examples in this textbook is **Ubuntu 20.04 LTS (Focal Fossa)** or **Ubuntu 22.04 LTS (Jammy Jellyfish)**. A fresh installation is ideal. If you are using Windows, consider setting up **WSL2 (Windows Subsystem for Linux 2)** with an Ubuntu distribution.

### 2. ROS 2 Humble Hawksbill Installation
Follow the official ROS 2 Humble installation guide for Ubuntu:
[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Verify your installation:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

### 3. Gazebo Fortress Installation
Gazebo Fortress is typically installed alongside ROS 2 Humble. Verify its installation:
```bash
ign gazebo
```
If not installed, follow the Gazebo installation guide for your ROS 2 version.

### 4. Python Environment
Ensure you are using the Python 3.x version compatible with your ROS 2 Humble installation (e.g., Python 3.8 or 3.10).
```bash
python3 --version
```
Install necessary Python packages (e.g., `rclpy` and other scientific libraries as needed for examples).
```bash
pip install -U rclpy # Check official ROS 2 docs for exact package names
```

### 5. NVIDIA Isaac Sim (Optional/Advanced)
For advanced sections involving NVIDIA Isaac Sim, you will need a compatible NVIDIA GPU and to download/install Isaac Sim from the NVIDIA Developer website. Refer to the official Isaac Sim documentation for system requirements and installation.
[https://developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)
