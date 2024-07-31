#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

sudo mkdir -p /tmp/test_results/optical_flow /tmp/log
sudo chown -R duckie /tmp/
sudo chown duckie /data/config/calibrations/camera_extrinsic/

# launching app
DEBUG=1 dt-exec rostest optical_flow optical_flow.test veh:=testdrone


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
