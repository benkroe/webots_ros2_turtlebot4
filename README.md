# Ubuntu VM Setup & Installation Guide (ROS 2 Humble)

This guide documents the steps to set up a new Ubuntu 22.04 ARM64 virtual machine in UTM, configure shared folders, enable remote access, and connect using VS Code for ROS 2 Humble.

## 1. Create a New VM in UTM
- Download the Ubuntu 22.04 ARM64 ISO (`ubuntu-22.04-live-server-arm64.iso`).
- In UTM, create a new virtual machine with all settings set to standard/default.

## 2. Set Up Shared Folder (macOS Host)
- Configure a shared folder between your macOS host and the Ubuntu VM using UTM's shared folder feature.

## 3. Enable SSH Access
- Boot into your Ubuntu VM.
- Update package lists and install OpenSSH server:
	```sh
	sudo apt update
	sudo apt install -y openssh-server
	```
- Ensure the SSH service is running:
	```sh
	sudo systemctl status ssh
	```

## 4. Connect to the VM with VS Code
- Use the Remote SSH extension in VS Code to connect to your Ubuntu VM using its IP address.

---

## 5. Install Webots 2025a (macOS host)
- Download and install Webots 2025a from the official website on your macOS host: [Webots Download](https://cyberbotics.com/#download)

## 6. Install ROS 2 Humble (Ubuntu VM)
Follow the official ROS 2 Humble installation guide: [ROS 2 Humble Ubuntu Install Debs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 6.1 Set Locale to UTF-8 (Ubuntu VM)
Check and set the system locale to UTF-8 on the VM:
```sh
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

### 6.2 Enable Universe Repository (Ubuntu VM)
```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 6.3 Add ROS 2 APT Repository (Ubuntu VM)
```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 6.4 Install Development Tools (Ubuntu VM)
```sh
sudo apt update && sudo apt install ros-dev-tools
```

### 6.5 Upgrade and Update Packages (Ubuntu VM)
```sh
sudo apt update
sudo apt upgrade
```

### 6.6 Install ROS 2 Humble Desktop (Ubuntu VM)
```sh
sudo apt install ros-humble-desktop
```

### 6.7 Source ROS 2 Setup Script (Ubuntu VM)
```sh
source /opt/ros/humble/setup.bash
```

---

Continue to add further steps as you proceed with the installation or configuration.

## 7. Webots Integration with ROS 2 (VM + host)
Follow the Webots installation guidance: [ROS 2 Humble Webots Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-MacOS.html)

### 7.1 Create Shared Folder Mount Point (Ubuntu VM)
```sh
mkdir -p /home/ubuntu/shared
```

### 7.2 Mount the macOS Shared Folder in the VM (Ubuntu VM)
Ensure the UTM shared folder name is `share` (default). Mount it inside the VM:
```sh
sudo mount -t 9p -o trans=virtio share /home/ubuntu/shared -oversion=9p2000.L
```

### 7.3 Export Webots Shared Folder Path (Ubuntu VM)
Set the environment variable to point to the host and VM shared paths. Replace `username` and the host shared path as needed:
```sh
export WEBOTS_SHARED_FOLDER=/Users/username/shared:/home/ubuntu/shared
```

### 7.4 Install Webots ROS 2 Packages (Ubuntu VM)
```sh
sudo apt-get install ros-humble-webots-ros2
```

### 7.5 Run Webots Local Simulation Server (macOS host)
Download the server script and run it with your Webots installation:
```sh
export WEBOTS_HOME=/Applications/Webots.app
curl -O https://raw.githubusercontent.com/cyberbotics/webots-server/main/local_simulation_server.py
python3 local_simulation_server.py
```

### 7.6 Validate Setup (Ubuntu VM)
Launch the sample multi-robot setup to verify connectivity and Webots integration:
```sh
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

---

## 8. Set Up Custom ROS 2 Workspace and Packages (Ubuntu VM)

### 8.1 Create ROS 2 Workspace
Create a workspace directory for your custom ROS 2 packages:
```sh
mkdir -p ~/ros2_ws
cd ~/ros2_ws
```

### 8.2 Clone Required Repositories
Clone the automodeROS and webots_ros2_turtlebot4 repositories:
```sh
git clone https://github.com/benkroe/automodeROS.git
git clone https://github.com/benkroe/webots_ros2_turtlebot4.git
```

### 8.3 Install Additional Dependencies (Ubuntu VM)
Install required message packages for Turtlebot4:
```sh
sudo apt install ros-humble-irobot-create-msgs
```

### 8.4 Build automodeROS Package
Navigate to the automodeROS directory and build with colcon:
```sh
cd ~/ros2_ws/automodeROS
colcon build --symlink-install
```

### 8.5 Build webots_ros2_turtlebot4 Package
Navigate to the webots_ros2_turtlebot4 directory and build:
```sh
cd ~/ros2_ws/webots_ros2_turtlebot4
colcon build --symlink-install
```

### 8.6 Source the Workspace
Source ROS 2 and the built packages:
```sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/automodeROS/install/setup.bash
source ~/ros2_ws/webots_ros2_turtlebot4/install/setup.bash
```

**Note:** Add these source commands to your `~/.bashrc` to automatically load them in new terminal sessions:
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/automodeROS/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/webots_ros2_turtlebot4/install/setup.bash" >> ~/.bashrc
```

### 8.7 Launch Turtlebot4 Simulation (Ubuntu VM)
Test the setup by launching the Turtlebot4 robot simulation:
```sh
ros2 launch webots_ros2_turtlebot4 robot_launch.py
```

Make sure the Webots simulation server is running on the macOS host (step 7.5) before launching.

---

## Notes on Humble vs Jazzy Differences

- **Ubuntu Version**: Humble requires Ubuntu 22.04 (Jammy) instead of 24.04 (Noble)
- **Python Version**: Uses Python 3.10 instead of 3.12
- **Executor**: Uses older, more stable executor implementation without the callback ordering changes in Jazzy
- **Performance**: Generally lower CPU usage for similar workloads compared to Jazzy
- **Repository Key**: Different method for adding ROS apt repository (uses ros.key instead of ros-apt-source package)

## Troubleshooting

### Clean Up Zombie Processes
If webots_ros2_driver processes persist after stopping the simulation:
```sh
pkill -9 -f webots_ros2_driver
```

### Auto-mount Shared Folder on Boot
Add to `/etc/fstab`:
```
share /home/ubuntu/shared 9p trans=virtio,version=9p2000.L,rw,_netdev,nofail 0 0
```
