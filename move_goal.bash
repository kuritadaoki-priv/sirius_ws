#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <goal_number>"
    exit 1
fi

ros2 run sirius_navigation move_goal $1
