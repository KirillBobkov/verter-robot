#!/bin/bash
# Setup script for ReSpeaker USB device permissions
# This script installs udev rules to grant access to the ReSpeaker microphone array

set -e

echo "🔧 Setting up ReSpeaker USB device permissions..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "❌ Please run this script with sudo"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RULES_FILE="$SCRIPT_DIR/99-respeaker-usb.rules"
UDEV_RULES_DIR="/etc/udev/rules.d"

# Check if the rules file exists
if [ ! -f "$RULES_FILE" ]; then
    echo "❌ Rules file not found: $RULES_FILE"
    exit 1
fi

# Copy the rules file to udev rules directory
echo "📋 Copying udev rules to $UDEV_RULES_DIR..."
cp "$RULES_FILE" "$UDEV_RULES_DIR/99-respeaker-usb.rules"

# Set proper permissions
chmod 644 "$UDEV_RULES_DIR/99-respeaker-usb.rules"

# Reload udev rules
echo "🔄 Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "✅ ReSpeaker USB permissions setup completed!"
echo ""
echo "📝 Please unplug and replug the ReSpeaker device for changes to take effect."
echo ""
echo "To verify the device is accessible, run:"
echo "  lsusb | grep 2886:0018"
