#!/bin/bash

# Script to install the Modulr Unitree Go2 systemd user service
# This will enable the ROS 2 nodes to start automatically on boot

set -e

echo "Installing Modulr Unitree Go2 systemd user service..."

# Create user systemd directory if it doesn't exist
mkdir -p ~/.config/systemd/user

# Copy service file to user systemd directory
cp "$(dirname "$0")/modulr-unitree-go2.service" ~/.config/systemd/user/

echo "Service file installed to ~/.config/systemd/user/modulr-unitree-go2.service"

# Reload systemd to recognize the new service
systemctl --user daemon-reload

echo "Systemd daemon reloaded"

# Enable the service to start on boot
systemctl --user enable modulr-unitree-go2.service

echo "Service enabled to start on boot"

# Enable lingering for the user (allows user services to run without login)
loginctl enable-linger $USER

echo "User lingering enabled (allows service to run without active login)"

echo ""
echo "Installation complete!"
echo ""
echo "To start the service now, run:"
echo "  systemctl --user start modulr-unitree-go2.service"
echo ""
echo "To check the service status, run:"
echo "  systemctl --user status modulr-unitree-go2.service"
echo ""
echo "To view logs, run:"
echo "  journalctl --user -u modulr-unitree-go2.service -f"
echo ""
echo "To disable the service, run:"
echo "  systemctl --user disable modulr-unitree-go2.service"
