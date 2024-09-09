#!/bin/bash

# This script publishes Imu, Range and Twist dummy messages to the respective topics
# subscribed by the state estimator node.

ROBOT_NAME="endeavour"

# Function to kill all child processes
cleanup() {
    echo "Shutting down all publishers..."
    kill 0
    exit
}

# Set up trap to call cleanup function when script receives SIGINT (CTRL+C)
trap cleanup SIGINT

# Start all publishers in the background
(
    # Publish Imu message
    rostopic pub /$ROBOT_NAME/state_estimator_node/imu sensor_msgs/Imu "{}" -r 50 &

    # Publish Range message
    rostopic pub -r 20 /$ROBOT_NAME/state_estimator_node/range sensor_msgs/Range \
    '{header: {stamp: now, frame_id: "base_link"}, radiation_type: 0, field_of_view: 0.1, min_range: 0.2, max_range: 10.0, range: 1.0}' &

    # Publish Twist message
    rostopic pub -r 10 /$ROBOT_NAME/state_estimator_node/twist geometry_msgs/TwistStamped "{}" &

    # Wait for all background processes
    wait
)

# Keep the script running
wait