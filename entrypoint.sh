#!/bin/zsh

# Print a message to indicate that the script is running
echo "Starting application..."
# Run the main application
source "/opt/ros/humble/setup.zsh"
source "~/.zshrc"

exec "$@"
