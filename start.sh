#!/bin/bash

# Logging untuk systemd
exec > >(logger -t robotpal) 2>&1

echo "===================================="
echo "Robot PAL Auto Start Service"
echo "Date: $(date)"
echo "User: $(whoami)"
echo "Working Directory: $(pwd)"
echo "===================================="

# Set domain for consistency
export ROS_DOMAIN_ID=0
echo "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"

# Hardware setup dengan sudo privileges
echo "Setting up CAN interface..."
if ip link show can0 2>/dev/null; then
    echo "CAN interface exists"
    if ip link show can0 | grep -q 'state DOWN'; then
        echo "Setting CAN interface up"
        sudo ip link set can0 up type can bitrate 250000
        if [ $? -eq 0 ]; then
            echo "CAN interface configured successfully"
        else
            echo "Failed to configure CAN interface"
        fi
    else
        echo 'CAN interface already up'
    fi
else
    echo "Warning: CAN interface can0 not found!"
fi

echo "Setting serial port permissions..."
for attempt in {1..5}; do
    echo "Attempt $attempt to set serial port permissions"
    
    if [ -e "/dev/ttyAMA0" ]; then
        echo "Serial port exists, setting permissions"
        sudo chmod 666 /dev/ttyAMA0
        sudo chown root:dialout /dev/ttyAMA0
        
        ls -la /dev/ttyAMA0
        
        if [ -r "/dev/ttyAMA0" ] && [ -w "/dev/ttyAMA0" ]; then
            echo "Successfully set serial port permissions"
            break
        else
            echo "Permission check failed, waiting before retry"
        fi
    else
        echo "WARNING: Serial port /dev/ttyAMA0 not found! Waiting for it to appear..."
    fi
    
    sleep 2
done

echo "Setting up ROS2 workspace..."
cd /home/pal/pal_ws

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "ROS2 jazzy sourced successfully"
else
    echo "ERROR: ROS2 jazzy not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Source workspace
echo "Sourcing workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Workspace sourced successfully"
else
    echo "ERROR: Workspace not built! install/setup.bash not found"
    exit 1
fi

# Verify ROS2 is working
echo "Verifying ROS2 environment..."
which ros2 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "ros2 command found: $(which ros2)"
else
    echo "ERROR: ros2 command not found in PATH"
    exit 1
fi

# Wait for system to be fully ready
echo "Waiting for system to be fully ready..."
sleep 15

echo "Starting full robot system..."
echo "Launch order: Device → Camera → Robot Controller"

# Launch the full system
echo "Executing: ros2 launch robot full_system.launch.py"
exec ros2 launch robot full_system.launch.py