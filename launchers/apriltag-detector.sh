#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec roslaunch duckietown_demos apriltag_detector.launch veh:=$VEHICLE_NAME
rosservice call --wait /$VEHICLE_NAME/apriltag_postprocessing_node/switch "{data: True}"
rosservice call --wait /$VEHICLE_NAME/apriltag_detector_node/switch "{data: True}"
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
