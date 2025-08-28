#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

# Run SSH check if interactive session
if [ -t 0 ]; then
    /home/ros/ssh-check.sh
fi

echo "Provided arguments: $@"

exec $@
